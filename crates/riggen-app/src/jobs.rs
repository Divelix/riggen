//! The job thread (docs/01-architecture.md §Jobs and threads): work that
//! is more than a frame's worth, run off the UI thread so the window keeps
//! painting.
//!
//! RoboCAD's `EvalExecutor` shape, which that section already prescribed —
//! a `std::thread`, an `mpsc` request/result pair, a `wake` callback bound
//! to `ctx.request_repaint()` so an idle UI does not sit on an undrained
//! result, and results drained once per frame. On wasm there is no thread:
//! [`Jobs::request`] runs the job inline and queues its result, so the
//! caller's code is identical either way.
//!
//! One job kind so far: a convex decomposition (ADR-0011), which is seconds
//! of work on a real part. Mesh loading and the export dialog's re-resolve
//! are still synchronous — backlog lines, not this.

use std::collections::HashSet;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, channel};

use riggen_core::MeshId;
use riggen_mesh::{DecompParams, TriMesh};

/// What identifies a job, so the same work is never queued twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum JobKey {
    Decompose(MeshId, DecompParams),
}

/// Work for the job thread.
pub enum Job {
    /// [`riggen_mesh::decompose`] of `source` — the mesh as the document
    /// wants it drawn, in meters — at `params`.
    Decompose {
        mesh: MeshId,
        params: DecompParams,
        source: Arc<TriMesh>,
    },
}

impl Job {
    pub fn key(&self) -> JobKey {
        match self {
            Self::Decompose { mesh, params, .. } => JobKey::Decompose(*mesh, *params),
        }
    }
}

/// What a finished job produced. Carries its own key, so a drain needs no
/// bookkeeping on the sending side.
pub enum JobResult {
    Decomposed {
        mesh: MeshId,
        params: DecompParams,
        /// The pieces, or why there are none (already a message: the UI
        /// shows it and `resolve` puts it in an `ExportError`).
        pieces: Result<Vec<Arc<TriMesh>>, String>,
    },
}

impl JobResult {
    pub fn key(&self) -> JobKey {
        match self {
            Self::Decomposed { mesh, params, .. } => JobKey::Decompose(*mesh, *params),
        }
    }
}

fn run(job: Job) -> JobResult {
    match job {
        Job::Decompose {
            mesh,
            params,
            source,
        } => JobResult::Decomposed {
            mesh,
            params,
            pieces: riggen_mesh::decompose(&source, &params)
                .map(|pieces| pieces.into_iter().map(Arc::new).collect())
                .map_err(|e| e.to_string()),
        },
    }
}

/// The app's handle on the job thread.
pub struct Jobs {
    /// `None` on wasm, where [`Self::request`] runs the job itself.
    requests: Option<Sender<Job>>,
    /// Kept on wasm so an inline result has somewhere to go.
    inline: Sender<JobResult>,
    results: Receiver<JobResult>,
    /// Requested and not yet drained. A second request for one of these is
    /// dropped rather than queued again.
    in_flight: HashSet<JobKey>,
}

impl Jobs {
    /// Starts the thread. `wake` runs on it whenever a result is queued —
    /// the app passes `ctx.request_repaint()`, so an idle window repaints
    /// and drains instead of waiting for a mouse move. Taken as a callback
    /// because nothing below `riggen-app` may know egui exists.
    pub fn new(wake: impl Fn() + Send + 'static) -> Self {
        let (outbox, results) = channel::<JobResult>();
        let inline = outbox.clone();

        #[cfg(target_arch = "wasm32")]
        {
            // No threads in the wasm build: `request` runs the job on the
            // spot. The channel and the drain stay, so app code does not
            // branch on the target.
            let _ = wake;
            Self {
                requests: None,
                inline,
                results,
                in_flight: HashSet::new(),
            }
        }

        #[cfg(not(target_arch = "wasm32"))]
        {
            let (requests, inbox) = channel::<Job>();
            std::thread::spawn(move || {
                // Ends when the app drops its `Sender`, i.e. at quit.
                while let Ok(job) = inbox.recv() {
                    if outbox.send(run(job)).is_err() {
                        break;
                    }
                    wake();
                }
            });
            Self {
                requests: Some(requests),
                inline,
                results,
                in_flight: HashSet::new(),
            }
        }
    }

    /// Queues `job` unless the same key is already in flight. Returns
    /// whether it was queued, which is only interesting to a test — the
    /// caller asks every frame and lets the deduplication do its work.
    pub fn request(&mut self, job: Job) -> bool {
        let key = job.key();
        if !self.in_flight.insert(key) {
            return false;
        }
        match &self.requests {
            Some(requests) => requests.send(job).is_ok(),
            // wasm: inline, so the result is already there on return.
            None => self.inline.send(run(job)).is_ok(),
        }
    }

    /// Whether `key` has been requested and not yet come back.
    pub fn is_pending(&self, key: JobKey) -> bool {
        self.in_flight.contains(&key)
    }

    /// Everything finished since the last call, in completion order. Called
    /// once per frame; the keys it returns stop being in flight.
    pub fn drain(&mut self) -> Vec<JobResult> {
        let done: Vec<JobResult> = self.results.try_iter().collect();
        for result in &done {
            self.in_flight.remove(&result.key());
        }
        done
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// `MeshId` has no public constructor; its `FromStr` is the way in.
    fn mesh_id(n: u32) -> MeshId {
        format!("m{n}").parse().unwrap()
    }

    fn cube(id: u32) -> (MeshId, Arc<TriMesh>) {
        (mesh_id(id), Arc::new(TriMesh::cube(0.05)))
    }

    /// The whole contract in one test: a request lands, `wake` fires, the
    /// drain returns it, and the key stops being in flight. No sleep — the
    /// loop blocks on the job's own result.
    #[test]
    fn a_decomposition_comes_back_and_wakes_the_ui() {
        let woken = Arc::new(AtomicUsize::new(0));
        let counter = woken.clone();
        let mut jobs = Jobs::new(move || {
            counter.fetch_add(1, Ordering::SeqCst);
        });
        let (mesh, source) = cube(1);
        let params = DecompParams::default();
        assert!(jobs.request(Job::Decompose {
            mesh,
            params,
            source,
        }));
        assert!(jobs.is_pending(JobKey::Decompose(mesh, params)));

        // "Pump frames" until the result lands: no timing assumption and
        // no sleep — an unfinished drain simply returns nothing, and the
        // loop ends on the job's own result.
        let pieces = loop {
            if let Some(JobResult::Decomposed { pieces, .. }) = jobs.drain().pop() {
                break pieces.unwrap();
            }
            std::thread::yield_now();
        };
        assert_eq!(pieces.len(), 1, "a cube is already convex");
        assert!(!jobs.is_pending(JobKey::Decompose(mesh, params)));
        assert!(woken.load(Ordering::SeqCst) >= 1, "the UI was never woken");
    }

    /// The same key twice is one job; a different key is another.
    #[test]
    fn requests_deduplicate_by_key_while_in_flight() {
        let mut jobs = Jobs::new(|| {});
        let (mesh, source) = cube(1);
        let params = DecompParams::default();
        let job = |mesh, params| Job::Decompose {
            mesh,
            params,
            source: source.clone(),
        };
        assert!(jobs.request(job(mesh, params)));
        assert!(!jobs.request(job(mesh, params)), "already in flight");

        let other = DecompParams {
            max_hulls: 3,
            ..params
        };
        assert!(jobs.request(job(mesh, other)), "different parameters");
        assert!(jobs.request(job(mesh_id(2), params)), "different mesh");

        let mut seen = HashSet::new();
        while seen.len() < 3 {
            seen.extend(jobs.drain().iter().map(JobResult::key));
        }
        assert_eq!(
            seen,
            HashSet::from([
                JobKey::Decompose(mesh, params),
                JobKey::Decompose(mesh, other),
                JobKey::Decompose(mesh_id(2), params),
            ])
        );
        // Drained, so the first key can be asked for again.
        assert!(jobs.request(job(mesh, params)));
    }

    /// A mesh with no decomposition comes back as a message, not a panic
    /// and not a missing result.
    #[test]
    fn a_degenerate_mesh_comes_back_as_an_error() {
        let mut jobs = Jobs::new(|| {});
        let plate = Arc::new(TriMesh {
            positions: vec![
                riggen_mesh::glam::DVec3::ZERO,
                riggen_mesh::glam::DVec3::X,
                riggen_mesh::glam::DVec3::Y,
            ],
            normals: Vec::new(),
            indices: vec![0, 1, 2],
        });
        jobs.request(Job::Decompose {
            mesh: mesh_id(1),
            params: DecompParams::default(),
            source: plate,
        });
        loop {
            if let Some(JobResult::Decomposed { pieces, .. }) = jobs.drain().pop() {
                assert!(pieces.unwrap_err().contains("no convex decomposition"));
                return;
            }
        }
    }
}
