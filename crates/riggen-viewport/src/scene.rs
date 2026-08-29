//! The per-instance scene: what the viewport draws, one entry per instance,
//! each with its own uploaded mesh, model transform and draw slot.
//!
//! Uploading one instance's mesh touches exactly that entry; showing,
//! hiding or moving one costs no upload at all. The `model` matrix is how a
//! joint preview or FK pose moves a link — by writing a transform, never by
//! re-uploading (docs/01-architecture.md §The document is the only state).

use riggen_mesh::glam::{DMat4, DVec3};
use riggen_mesh::{Aabb, TriMesh};

/// Identifies one drawable thing in the viewport. Handed out by the app;
/// the viewport never invents one. Stable for the instance's lifetime and
/// never reused, which is why the pick id carries a recycled *slot* instead
/// ([`crate::pick_id`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct InstanceId(pub u32);

/// How many instances a scene can hold at once: the pick id has
/// [`crate::pick_id::SLOT_BITS`] bits for the slot.
pub const MAX_INSTANCES: usize = 1 << crate::pick_id::SLOT_BITS;

/// `set_instance` on a scene with every slot taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SceneFull;

impl std::fmt::Display for SceneFull {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "the viewport holds at most {MAX_INSTANCES} instances")
    }
}

impl std::error::Error for SceneFull {}

/// How a [`TriMesh`] becomes whatever a [`Scene`] holds per instance.
///
/// The real implementation is [`crate::GpuMesh`] (`Context = wgpu::Device`);
/// the indirection is what lets `Scene`'s bookkeeping — the part with the
/// invariants worth testing — be exercised without a GPU.
pub trait InstancePayload: Sized {
    type Context: ?Sized;

    /// Uploads `mesh` for the instance in draw `slot`; the slot is baked into
    /// the pick vertices, which is why it is passed in here.
    fn upload(ctx: &Self::Context, slot: u32, mesh: &TriMesh) -> Self;
}

/// The colour an instance draws with until told otherwise: M0's blue-grey.
/// Linear RGBA, multiplied by the shader's lighting.
pub const DEFAULT_INSTANCE_COLOR: [f32; 4] = [0.55, 0.65, 0.78, 1.0];

/// Which pass draws an instance. Opaque instances draw first and write
/// depth; translucent ones draw after every opaque one, alpha-blended and
/// depth-tested without writing depth, and the pick pass skips them — a
/// collision hull over a part must not steal the part's clicks
/// (docs/01-architecture.md §Frame loop).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RenderGroup {
    #[default]
    Opaque,
    Translucent,
}

/// One instance: its uploaded payload, where it sits, and whether it draws.
pub struct InstanceEntry<M> {
    pub key: InstanceId,
    /// Pick-id slot, unique among live instances and kept across re-uploads.
    pub slot: u32,
    pub mesh: M,
    pub model: DMat4,
    /// Linear RGBA; the material tint.
    pub color: [f32; 4],
    pub visible: bool,
    pub group: RenderGroup,
    /// Model-space bounds, kept so zoom-to-fit never needs the CPU
    /// positions back.
    pub bounds: Option<Aabb>,
}

/// Every instance currently in the viewport, in insertion order (which is
/// also draw order and the order [`Scene::visible`] hands them to the
/// render pass).
pub struct Scene<M> {
    instances: Vec<InstanceEntry<M>>,
    /// Slots released by `remove`, reused before new ones are minted.
    free_slots: Vec<u32>,
    next_slot: u32,
}

impl<M> Default for Scene<M> {
    fn default() -> Self {
        Self {
            instances: Vec::new(),
            free_slots: Vec::new(),
            next_slot: 0,
        }
    }
}

impl<M: InstancePayload> Scene<M> {
    /// Uploads `mesh` as `id`'s geometry, replacing whatever it held before
    /// and leaving every *other* instance's buffers untouched. An instance
    /// that already exists keeps its slot, model transform and visibility,
    /// so re-uploading a hidden or placed instance doesn't un-hide or reset
    /// it.
    pub fn set_instance(
        &mut self,
        ctx: &M::Context,
        id: InstanceId,
        mesh: &TriMesh,
    ) -> Result<(), SceneFull> {
        let bounds = mesh.aabb();
        if let Some(entry) = self.instances.iter_mut().find(|e| e.key == id) {
            entry.mesh = M::upload(ctx, entry.slot, mesh);
            entry.bounds = bounds;
            return Ok(());
        }
        let slot = match self.free_slots.pop() {
            Some(slot) => slot,
            None if (self.next_slot as usize) < MAX_INSTANCES => {
                self.next_slot += 1;
                self.next_slot - 1
            }
            None => return Err(SceneFull),
        };
        self.instances.push(InstanceEntry {
            key: id,
            slot,
            mesh: M::upload(ctx, slot, mesh),
            model: DMat4::IDENTITY,
            color: DEFAULT_INSTANCE_COLOR,
            visible: true,
            group: RenderGroup::default(),
            bounds,
        });
        Ok(())
    }
}

impl<M> Scene<M> {
    /// Drops an instance, its buffers and its slot. `true` if it was there.
    pub fn remove(&mut self, id: InstanceId) -> bool {
        let Some(index) = self.instances.iter().position(|e| e.key == id) else {
            return false;
        };
        let entry = self.instances.remove(index);
        self.free_slots.push(entry.slot);
        true
    }

    /// Shows or hides an instance without touching its buffers.
    pub fn set_visible(&mut self, id: InstanceId, visible: bool) -> bool {
        match self.instances.iter_mut().find(|e| e.key == id) {
            Some(entry) => {
                entry.visible = visible;
                true
            }
            None => false,
        }
    }

    /// Places an instance, again without touching its buffers.
    pub fn set_model(&mut self, id: InstanceId, model: DMat4) -> bool {
        match self.instances.iter_mut().find(|e| e.key == id) {
            Some(entry) => {
                entry.model = model;
                true
            }
            None => false,
        }
    }

    /// Sets the tint. `false` if `id` is not in the scene.
    pub fn set_color(&mut self, id: InstanceId, color: [f32; 4]) -> bool {
        match self.instances.iter_mut().find(|e| e.key == id) {
            Some(entry) => {
                entry.color = color;
                true
            }
            None => false,
        }
    }

    /// Moves an instance between the opaque and translucent passes. No
    /// upload. `false` if `id` is not in the scene.
    pub fn set_group(&mut self, id: InstanceId, group: RenderGroup) -> bool {
        match self.instances.iter_mut().find(|e| e.key == id) {
            Some(entry) => {
                entry.group = group;
                true
            }
            None => false,
        }
    }

    pub fn clear(&mut self) {
        self.instances.clear();
        self.free_slots.clear();
        self.next_slot = 0;
    }

    pub fn contains(&self, id: InstanceId) -> bool {
        self.instances.iter().any(|e| e.key == id)
    }

    pub fn get(&self, id: InstanceId) -> Option<&InstanceEntry<M>> {
        self.instances.iter().find(|e| e.key == id)
    }

    /// Every instance, in draw order, hidden ones included — unlike
    /// [`Self::visible`], which is what the render pass walks.
    pub fn entries(&self) -> impl Iterator<Item = &InstanceEntry<M>> + '_ {
        self.instances.iter()
    }

    pub fn keys(&self) -> impl Iterator<Item = InstanceId> + '_ {
        self.instances.iter().map(|e| e.key)
    }

    pub fn len(&self) -> usize {
        self.instances.len()
    }

    pub fn is_empty(&self) -> bool {
        self.instances.is_empty()
    }

    /// The instances that actually draw this frame, in scene order. Their
    /// position in this iterator is the index every model-uniform offset is
    /// stated in.
    pub fn visible(&self) -> impl Iterator<Item = &InstanceEntry<M>> + '_ {
        self.instances.iter().filter(|e| e.visible)
    }

    /// The visible instance `id`, paired with its index among the visible
    /// ones — "which buffers, at which draw slot".
    pub fn visible_instance(&self, id: InstanceId) -> Option<(usize, &InstanceEntry<M>)> {
        self.visible().enumerate().find(|(_, e)| e.key == id)
    }

    /// The instance whose pick vertices carry `slot` — how a decoded pick
    /// id becomes an [`InstanceId`]. `None` for a slot nobody holds (a
    /// stale readback from an instance removed since the pass was issued).
    pub fn instance_at_slot(&self, slot: u32) -> Option<InstanceId> {
        self.instances
            .iter()
            .find(|e| e.slot == slot)
            .map(|e| e.key)
    }

    /// Bounding sphere `(center, radius)` of every visible instance, for
    /// zoom-to-fit. The radius is the union box's half-diagonal rather than
    /// the exact farthest point, since the scene keeps per-instance boxes
    /// instead of the CPU positions — a fit that is never too tight.
    pub fn bounds(&self) -> Option<(DVec3, f64)> {
        let union = self
            .visible()
            .filter_map(|e| e.bounds.map(|b| b.transformed(&e.model)))
            .reduce(|a, b| a.union(&b))?;
        Some((union.center(), union.half_diagonal()))
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;

    use super::*;

    /// Counts uploads instead of touching a GPU: the invariant under test
    /// is *how many* instances a scene edit re-uploads, which is
    /// bookkeeping, not rendering.
    #[derive(Default)]
    struct Uploads(Cell<usize>);

    struct TestPayload {
        slot: u32,
        triangles: usize,
    }

    impl InstancePayload for TestPayload {
        type Context = Uploads;

        fn upload(ctx: &Uploads, slot: u32, mesh: &TriMesh) -> Self {
            ctx.0.set(ctx.0.get() + 1);
            TestPayload {
                slot,
                triangles: mesh.triangle_count(),
            }
        }
    }

    fn scene_of(ctx: &Uploads, n: u32) -> Scene<TestPayload> {
        let mut scene = Scene::default();
        for i in 0..n {
            scene
                .set_instance(ctx, InstanceId(i), &TriMesh::cube(1.0))
                .unwrap();
        }
        scene
    }

    fn two_triangles() -> TriMesh {
        let mut mesh = TriMesh::cube(1.0);
        mesh.indices.truncate(6);
        mesh
    }

    #[test]
    fn reuploading_one_instance_uploads_only_that_instance() {
        let ctx = Uploads::default();
        let mut scene = scene_of(&ctx, 4);
        assert_eq!(ctx.0.get(), 4, "four instances, four uploads");

        scene
            .set_instance(&ctx, InstanceId(2), &two_triangles())
            .unwrap();

        assert_eq!(ctx.0.get(), 5, "the re-uploaded instance, and nothing else");
        assert_eq!(scene.visible().count(), 4, "no instance was duplicated");
        let (_, entry) = scene
            .visible_instance(InstanceId(2))
            .expect("still in the scene");
        assert_eq!(
            entry.mesh.triangles, 2,
            "and it is the new mesh that landed"
        );
        assert_eq!(entry.mesh.slot, entry.slot, "stamped with its own slot");
    }

    #[test]
    fn visibility_and_placement_upload_nothing() {
        let ctx = Uploads::default();
        let mut scene = scene_of(&ctx, 3);

        assert!(scene.set_visible(InstanceId(1), false));
        assert!(scene.set_model(InstanceId(2), DMat4::from_translation(DVec3::X)));
        assert!(!scene.set_visible(InstanceId(9), false), "unknown id");
        assert!(!scene.set_model(InstanceId(9), DMat4::IDENTITY));

        assert_eq!(ctx.0.get(), 3, "still just the three initial uploads");
        assert_eq!(
            scene.visible().count(),
            2,
            "the hidden instance stopped drawing"
        );
        assert!(scene.visible_instance(InstanceId(1)).is_none());
        assert!(scene.contains(InstanceId(1)), "hidden, not gone");
    }

    /// A new mesh arriving for a hidden or placed instance must not quietly
    /// un-hide or re-center it, nor change its slot.
    #[test]
    fn reuploading_preserves_visibility_model_and_slot() {
        let ctx = Uploads::default();
        let mut scene = scene_of(&ctx, 2);
        let model = DMat4::from_translation(DVec3::new(0.0, 2.0, 0.0));
        scene.set_visible(InstanceId(0), false);
        scene.set_model(InstanceId(0), model);
        let slot = scene.get(InstanceId(0)).unwrap().slot;

        scene
            .set_instance(&ctx, InstanceId(0), &two_triangles())
            .unwrap();

        assert!(
            scene.visible_instance(InstanceId(0)).is_none(),
            "still hidden"
        );
        let entry = scene.get(InstanceId(0)).unwrap();
        assert!(!entry.visible);
        assert_eq!(entry.model, model);
        assert_eq!(entry.slot, slot);
        assert_eq!(entry.mesh.slot, slot);
        assert_eq!(entry.mesh.triangles, 2, "but the new mesh did land");
    }

    #[test]
    fn group_is_bookkeeping_that_survives_a_reupload() {
        let ctx = Uploads::default();
        let mut scene = scene_of(&ctx, 2);
        assert_eq!(scene.get(InstanceId(1)).unwrap().group, RenderGroup::Opaque);
        assert!(scene.set_group(InstanceId(1), RenderGroup::Translucent));
        assert!(!scene.set_group(InstanceId(9), RenderGroup::Translucent));
        scene
            .set_instance(&ctx, InstanceId(1), &two_triangles())
            .unwrap();
        assert_eq!(
            scene.get(InstanceId(1)).unwrap().group,
            RenderGroup::Translucent
        );
        assert_eq!(ctx.0.get(), 3, "the group change uploaded nothing");
        // Still in the visible order: the translucent pass draws from the
        // same list, after the opaque one.
        assert_eq!(scene.visible().count(), 2);
    }

    #[test]
    fn removing_an_instance_drops_it_from_the_draw_order() {
        let ctx = Uploads::default();
        let mut scene = scene_of(&ctx, 3);

        assert!(scene.remove(InstanceId(1)));
        assert!(!scene.remove(InstanceId(1)), "already gone");

        assert_eq!(ctx.0.get(), 3, "removal uploads nothing either");
        let keys: Vec<InstanceId> = scene.keys().collect();
        assert_eq!(keys, vec![InstanceId(0), InstanceId(2)]);
        assert_eq!(scene.len(), 2);
    }

    /// Visible-order indices are what the render pass binds uniforms by, so
    /// hiding an instance has to renumber the ones after it.
    #[test]
    fn visible_index_skips_hidden_instances() {
        let ctx = Uploads::default();
        let mut scene = scene_of(&ctx, 3);
        assert_eq!(
            scene.visible_instance(InstanceId(2)).map(|(i, _)| i),
            Some(2)
        );

        scene.set_visible(InstanceId(0), false);

        assert_eq!(
            scene.visible_instance(InstanceId(2)).map(|(i, _)| i),
            Some(1)
        );
    }

    #[test]
    fn bounds_cover_every_visible_instance_and_follow_its_model() {
        let ctx = Uploads::default();
        let mut scene: Scene<TestPayload> = Scene::default();
        let unit = TriMesh::cube(1.0);
        scene.set_instance(&ctx, InstanceId(0), &unit).unwrap();
        scene.set_instance(&ctx, InstanceId(1), &unit).unwrap();
        scene.set_model(
            InstanceId(1),
            DMat4::from_translation(DVec3::new(8.0, 0.0, 0.0)),
        );

        let (center, radius) = scene.bounds().expect("two instances");
        assert!((center.x - 4.0).abs() < 1e-9, "midway between them");
        // Union box spans x ∈ [-1, 9], y, z ∈ [-1, 1]: half-diagonal of (10, 2, 2).
        assert!((radius - (25.0f64 + 1.0 + 1.0).sqrt()).abs() < 1e-9);

        scene.set_visible(InstanceId(1), false);
        let (center, radius) = scene.bounds().expect("one instance left");
        assert!(center.x.abs() < 1e-9, "back to the instance at the origin");
        assert!((radius - 3f64.sqrt()).abs() < 1e-9);

        scene.set_visible(InstanceId(0), false);
        assert_eq!(scene.bounds(), None, "nothing visible, nothing to fit");
    }

    /// Slots are what the pick vertices carry, so they must be unique among
    /// live instances, recycled after removal, and resolvable back to an id.
    #[test]
    fn slots_are_unique_recycled_and_resolve_to_ids() {
        let ctx = Uploads::default();
        let mut scene = scene_of(&ctx, 3);
        let slots: Vec<u32> = scene.entries().map(|e| e.slot).collect();
        assert_eq!(slots, vec![0, 1, 2]);
        assert_eq!(scene.instance_at_slot(1), Some(InstanceId(1)));
        assert_eq!(scene.instance_at_slot(7), None);

        scene.remove(InstanceId(1));
        assert_eq!(
            scene.instance_at_slot(1),
            None,
            "released with its instance"
        );

        scene
            .set_instance(&ctx, InstanceId(40), &TriMesh::cube(1.0))
            .unwrap();
        let entry = scene.get(InstanceId(40)).unwrap();
        assert_eq!(entry.slot, 1, "the freed slot is reused before a new one");
        assert_eq!(entry.mesh.slot, 1);
        assert_eq!(scene.instance_at_slot(1), Some(InstanceId(40)));

        scene
            .set_instance(&ctx, InstanceId(41), &TriMesh::cube(1.0))
            .unwrap();
        assert_eq!(
            scene.get(InstanceId(41)).unwrap().slot,
            3,
            "then a fresh one"
        );
    }

    #[test]
    fn a_full_scene_refuses_new_instances_but_not_reuploads() {
        let ctx = Uploads::default();
        let mut scene = scene_of(&ctx, MAX_INSTANCES as u32);
        let cube = TriMesh::cube(1.0);
        assert_eq!(
            scene.set_instance(&ctx, InstanceId(u32::MAX), &cube),
            Err(SceneFull)
        );
        assert_eq!(scene.set_instance(&ctx, InstanceId(5), &cube), Ok(()));
        scene.remove(InstanceId(5));
        assert_eq!(
            scene.set_instance(&ctx, InstanceId(u32::MAX), &cube),
            Ok(())
        );

        scene.clear();
        assert!(scene.is_empty());
        assert_eq!(scene.set_instance(&ctx, InstanceId(0), &cube), Ok(()));
        assert_eq!(scene.get(InstanceId(0)).unwrap().slot, 0, "slots restart");
    }
}
