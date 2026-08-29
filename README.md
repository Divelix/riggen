# riggen

**The blazingly fast, lightweight robot assembler for RL researchers.**

Drop STL/OBJ meshes into a native GPU window, build the kinematic tree,
place joints by clicking the bore, compute sim-ready inertials and
collision geometry, export MJCF and URDF — and import a URDF to fix and
convert it. Rust + egui + wgpu, shipped as a Python wheel.

```sh
uv tool install riggen && riggen
```

Development happens in the open at <https://github.com/Divelix/riggen>.

Licensed under MIT or Apache-2.0, at your option.
