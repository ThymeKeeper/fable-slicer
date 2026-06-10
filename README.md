# slicer (working name)

A from-scratch FDM 3D-printing slicer in Rust. Clean reimplementation — **not** a
fork of the Slic3r / PrusaSlicer / Bambu / Orca lineage. Algorithms are learned
from papers and from reading reference engines (CuraEngine), then written fresh
in idiomatic Rust.

> Status: **M5/M6** — STL in, Klipper g-code out, desktop GUI with a 3D bead
> preview. Walls, solid shells with monotonic fill + ironing, five infill
> patterns (incl. gyroid), gap fill, fuzzy skin, spiral vase, supports
> (grid / **arc overhangs** / bridges), **brick layering**, combing + z-hop,
> seam control, G2/G3 arc fitting, per-feature speeds & fan, pressure advance,
> elephant-foot/XY compensation, M73 progress, print-time/filament estimates.
> Layer-parallel (rayon) with z-bucketed slicing: a 225k-triangle Benchy plans
> in ~0.3s. See [PLAN.md](PLAN.md) for the roadmap and
> [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the design.

## Quick start

```sh
# The GUI is the product — bare `cargo run` launches it: import STLs, lay out
# the bed, tune settings, slice, preview in 3D, save profiles, export g-code.
cargo run --release

# Run the test suite (includes a cube-slicing acceptance test).
cargo test --workspace

# CLI: generate the cube fixture, then slice it to g-code (+ optional SVGs).
cargo run -p mesh --example gen_cube
cargo run -p cli -- fixtures/cube.stl --printer voron24 -o cube.gcode --svg svg/
# -> cube.gcode, plus svg/layer_*.svg to inspect walls / shells / infill.

# Pick profiles (list them with --list-profiles); flags like --layer-height
# override. User profiles saved from the GUI are picked up automatically —
# see docs/PROFILES.md for the tier/inheritance model.
cargo run -p cli -- model.stl --printer voron24 --filament petg --process fine -o out.gcode

# Feature flags: gyroid infill, fuzzy skin, ironing, spiral vase, arc overhangs…
cargo run -p cli -- model.stl --sparse-infill gyroid --ironing -o out.gcode
cargo run -p cli -- vase.stl --vase -o vase.gcode
cargo run -p cli -- model.stl --support arc --arc-fitting -o out.gcode

# Pipeline timing on a model (load / slice / plan / g-code):
cargo run --release -p engine --example bench -- fixtures/benchy.stl
```

## Workspace

| crate    | role |
|----------|------|
| `geo2d`  | integer 2D geometry (Clipper-space points / contours / polygons) |
| `mesh`   | triangle mesh + STL I/O |
| `engine` | the slicer core (slicing, walls, solid/sparse infill, toolpaths, g-code) |
| `gcode`  | low-level g-code emitter (relative E, retraction, temps/fan) |
| `config` | printer/filament/process profiles: built-in + user, TOML, inheritance ([docs](docs/PROFILES.md)) |
| `gui`    | **the primary front-end** — egui + wgpu 3D viewport (binary: `slicer`, default for `cargo run`) |
| `cli`    | command-line front-end (binary: `slicer-cli`) |

## License

AGPL-3.0-or-later. (Porting algorithms from AGPL reference engines makes the
result AGPL; this is an accepted, deliberate choice — see the decision log in
PLAN.md.)
