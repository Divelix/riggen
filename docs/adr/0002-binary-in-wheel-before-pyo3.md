# ADR-0002: ship the binary in the wheel; PyO3 only for the headless SDK

- Status: Accepted; amended by ADR-0009 (2026-08-30) — the v0.2 layout (one
  wheel: abi3 extension + the binary as wheel data), and the open question
  below closes as "no"
- Date: 2026-08-29

## Context

The product promise is `pip install riggen` → `riggen` opens a native window.
The seed document said "PyO3 and maturin to wrap the Rust binary", which
reads as an extension module that opens the GUI from inside the interpreter.
Rerun, the reference for this experience, does *not* do that: its wheel
carries the native `rerun` executable and `rerun_cli/__main__.py` is a
30-line `subprocess.call` to it; the PyO3 module (`rerun_bindings`) is the
logging SDK, not the viewer.

Opening an eframe window from inside a Python process means the GIL, Python's
signal handlers, and macOS's main-thread requirement all become our problem,
and the GUI can never outlive the interpreter call.

## Decision

- MVP (M4): maturin `bindings = "bin"`. The wheel contains the `riggen`
  executable and a console-script entry point; `python -m riggen` execs it.
  No PyO3 in the critical path.
- v0.2: `riggen-py`, a PyO3 `cdylib` over `riggen-core` and `riggen-export`
  only — build, validate, FK, export, import — with `riggen.show(robot)`
  serialising to a temp `.riggen` and spawning the bundled binary. The GUI
  is never entered from inside a Python call.

## Consequences

- Distribution risk retires at M4 with zero binding code; the UX risk (M2)
  gets the time instead.
- `riggen-core` and `riggen-export` must stay free of egui/wgpu so the SDK can
  link them; the layer rule in 01-architecture is what enforces it.
- Two build artefacts per platform in v0.2 (bin + cdylib); CI handles it the
  way Rerun's does.
- Users who want a live link between a running script and the GUI (log joint
  states, see them move) get it later via a file/socket, not via a shared
  process. ⚠ OPEN: revisit if that becomes the headline SDK feature.

## Alternatives considered

- **PyO3 extension module that runs eframe in-process** — the problems above;
  also forces every GUI dependency into the module and doubles the wheel's
  ABI surface.
- **Pure-Rust distribution (`cargo install`, GitHub releases)** — keep as a
  secondary channel; it misses the audience that has `uv` and not `cargo`.
