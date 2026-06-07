# slicer (working name)

A from-scratch FDM 3D-printing slicer in Rust. Clean reimplementation — **not** a
fork of the Slic3r / PrusaSlicer / Bambu / Orca lineage. Algorithms are learned
from papers and from reading reference engines (CuraEngine), then written fresh
in idiomatic Rust.

> Status: **M2** — STL in, Klipper g-code out: walls, solid top/bottom shells,
> sparse infill, retraction, bed-centering. See [PLAN.md](PLAN.md) for the roadmap
> and [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the design.

## Quick start

```sh
# Run the test suite (includes a cube-slicing acceptance test).
cargo test

# Generate the cube fixture, then slice it to g-code (+ optional toolpath SVGs).
cargo run -p mesh --example gen_cube
cargo run --bin slicer -- fixtures/cube.stl --printer voron24 -o cube.gcode --svg svg/
# -> cube.gcode, plus svg/layer_*.svg to inspect walls / shells / infill.

# Slice your own model:
cargo run --bin slicer -- path/to/model.stl --printer voron24 -o out.gcode

# Pick profiles (list them with --list-profiles); flags like --layer-height override:
cargo run --bin slicer -- model.stl --printer voron24 --filament petg --process fine -o out.gcode
```

## Workspace

| crate    | role |
|----------|------|
| `geo2d`  | integer 2D geometry (Clipper-space points / contours / polygons) |
| `mesh`   | triangle mesh + STL I/O |
| `engine` | the slicer core (slicing, walls, solid/sparse infill, toolpaths, g-code) |
| `gcode`  | low-level g-code emitter (relative E, retraction, temps/fan) |
| `config` | tiered printer/filament/process profiles (TOML, inheritance) + resolved settings |
| `cli`    | command-line front-end (binary: `slicer`) |

## License

AGPL-3.0-or-later. (Porting algorithms from AGPL reference engines makes the
result AGPL; this is an accepted, deliberate choice — see the decision log in
PLAN.md.)
