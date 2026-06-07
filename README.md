# slicer (working name)

A from-scratch FDM 3D-printing slicer in Rust. Clean reimplementation — **not** a
fork of the Slic3r / PrusaSlicer / Bambu / Orca lineage. Algorithms are learned
from papers and from reading reference engines (CuraEngine), then written fresh
in idiomatic Rust.

> Status: **M0** — mesh in, per-layer polygons out. See [PLAN.md](PLAN.md) for the
> roadmap and [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the design.

## Quick start

```sh
# Run the test suite (includes a cube-slicing acceptance test).
cargo test

# Generate the cube fixture, then slice it to per-layer SVGs.
cargo run -p mesh --example gen_cube
cargo run --bin slicer -- fixtures/cube.stl --layer-height 0.2 --out out
# -> out/layer_0000.svg ... open any of them to inspect the slice.

# Slice your own model:
cargo run --bin slicer -- path/to/model.stl --layer-height 0.2 --out out
```

## Workspace

| crate    | role |
|----------|------|
| `geo2d`  | integer 2D geometry (Clipper-space points / contours / polygons) |
| `mesh`   | triangle mesh + STL I/O |
| `engine` | the slicer core (slicing now; walls/infill/toolpaths later) |
| `gcode`  | g-code AST + writer + time estimate (stub until M1) |
| `config` | printer/filament/process profiles (stub until M2) |
| `cli`    | command-line front-end (binary: `slicer`) |

## License

AGPL-3.0-or-later. (Porting algorithms from AGPL reference engines makes the
result AGPL; this is an accepted, deliberate choice — see the decision log in
PLAN.md.)
