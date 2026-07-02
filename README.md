<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/a2f4ff3c-3402-451b-8259-bbc7f6a9b606" />

<img width="2880" height="1800" alt="Screenshot from 2026-06-28 09-42-03" src="https://github.com/user-attachments/assets/e4605eb6-817c-46de-b89b-7e0f696158ab" />

# Fable Slicer

Fable Slicer is a from-scratch FDM 3D-printing slicer written in Rust. It takes STL and 3MF models in and produces Klipper-flavored G-code out. The desktop GUI is the primary interface — a warm dark egui application with a custom wgpu 3D viewport for arranging models, slicing, previewing toolpaths, and sending jobs to a printer — and a command-line front-end is also available for scripted and headless use. Under both front-ends is a headless engine library that runs the full pipeline: mesh → layers → walls → infill → toolpaths → G-code.

## Features

### Slicing
- Mesh slicing by horizontal-plane intersection, parallelized with rayon, with triangle bucketing by z-span and integer/fixed-point XY coordinates
- Contour stitching with orientation by nesting depth (CCW outers, CW holes)
- Distinct first-layer height and layer height

### Walls, skins & infill
- Concentric perimeter walls: an outer wall that hugs the true outline plus inner walls at correct bead spacing; configurable wall count and outer-wall-first ordering
- Top and bottom solid skins via boolean coverage across neighboring layers, split into bottom skin, buried solid, and top skin
- Sparse infill patterns: Lines, Aligned Lines, Grid, Triangles, Concentric, and Gyroid (a 3D level-set that drifts with layer height so layers interlock); independent pattern choice for sparse, top, bottom, and solid regions
- Configurable infill density and wall overlap; solid/sparse rebalancing and Concentric fill for narrow ring-bands
- Monotonic ordering option for top/bottom surfaces

### Bridging & supports
- Bridge detection over unsupported islands anchored on two or more sides, spanning the shortest of several tested angles up to a maximum span, plus enclosed-ceiling sheet bridging with a foothold
- Grid supports: overhang detection by angle, downward projection, z-gap and solid interface layers, XY clearance, perimeter loop plus line fill

### Surface & quality
- Spiral vase mode (single continuous Z ramp, one wall, no infill or shells)
- Fuzzy skin (deterministic jitter of the outer perimeter)
- Ironing (low-flow, fine-spaced pass over exposed top surfaces)
- Elephant-foot compensation and all-layer XY compensation
- Seam placement: Nearest, Sharpest, Random, and Aligned (cross-layer seam tracking)
- Skirt and brim for priming and adhesion

### Motion & output
- Stadium-bead extrusion model with flow clamping against a max volumetric speed
- Arc fitting (G2/G3 emission) with a configurable tolerance
- Combing travel via a per-layer visibility graph with Dijkstra routing, falling back to retraction plus z-hop; wipe-on-retract
- Per-segment bead attributes that retune speed, flow, acceleration, pressure advance, and fan across overhang and bridge stretches without splitting a bead
- Per-feature speeds, accelerations, pressure advance, and fan control; minimum-layer-time cooling
- Klipper-targeted G-code: relative extrusion (M83), per-feature acceleration (M204), pressure advance, velocity limits, progress (M73), start/end template substitution, chamber pre-soak, and aux/exhaust fan control
- Trapezoidal print-time estimate (with jerk and look-ahead) and filament estimate in millimeters and grams
- Guided flow calibration: prints a single-wall test cube and derives an extrusion multiplier from the measured wall thickness

### GUI
- Import STL and 3MF (a 3MF splits into one object per build item, preserving plate layout)
- Multi-object, multi-bed scene: place, rotate, scale, duplicate, drag, and auto-arrange objects across beds
- Orbit/pan/zoom camera; Model and Preview views
- Toolpath preview colored by feature type or by layer time (heat ramp), with per-category toggles and a vertical layer slider
- Full three-tier profile selector with per-tier dirty indicators, save/delete of user profiles, and auto/pin derived settings
- Spiral-vase lockouts, arc-fitting toggle, and accent-color theming
- Headless one-shot PNG render of a preview layer via `--render-layer`

## Building & running

Fable Slicer is a Cargo workspace and needs a Rust toolchain (edition 2021).

The GUI is the default workspace member, so a bare `cargo run` launches it:

```sh
cargo run                       # run the GUI (debug)
cargo build --release           # release binary at target/release/fable-slicer
```

The compiled GUI binary is named `fable-slicer`.

The CLI is the secondary front-end. Its Cargo package is `cli` but the compiled binary is `fable-slicer-cli`:

```sh
cargo run -p cli -- model.stl -o out.gcode
cargo build --release --bin fable-slicer-cli
```

Example CLI invocations:

```sh
# Slice with explicit profiles and a few overrides
fable-slicer-cli model.stl --printer sovol-zero --filament petg --process standard \
    --walls 3 --infill 0.2 --seam aligned -o out.gcode

# List available profiles and exit
fable-slicer-cli --list-profiles

# Dump per-layer toolpath SVGs while slicing
fable-slicer-cli model.stl --svg ./svg_out
```

Run the full test suite across all crates with `cargo test --workspace`.

## Profiles

Settings are organized into three tiers, each field owned by exactly one tier:

- **Printer** — machine geometry, motion limits, retraction, arc fitting, host connection, and start/end G-code
- **Filament** — material class, temperatures, cooling, flow, pressure advance, and bridge settings
- **Process** — layer heights, walls, infill, supports, seams, surface features, and the speed/quality dial

Profiles use single-parent inheritance (`inherits = "name"`), resolved root-first with the child overriding its parent. Built-in profiles are embedded read-only; user profiles are saved as minimal diffs (only the fields you changed).

Built-in profiles:

- **Printers:** `generic`, `voron24`, `sovol-zero`
- **Filaments:** `pla`, `petg`, `pla-hf`, `asa`, `abs`, `polymaker-pc`
- **Processes:** `standard`

User profiles live under the per-OS config directory in `fable-slicer/profiles/{printer,filament,process}/*.toml`:

- Linux: `~/.config/fable-slicer/`
- macOS: `~/Library/Application Support/fable-slicer/`
- Windows: `%APPDATA%\fable-slicer\`

List everything available with `fable-slicer-cli --list-profiles`. A user profile whose name collides with a built-in is skipped with a warning; `--profile-dir <dir>` loads an extra directory that is allowed to shadow built-ins.

## Printer integration

Both front-ends can talk to a Moonraker/Klipper host. The GUI offers a connection test, upload, upload-and-print, and a live-print card with a progress bar and pause/resume/cancel. The CLI exposes `--upload`, `--start-print`, and `--host` (falling back to the printer profile's host URL). Before sending, Fable Slicer runs a chamber-sensor pre-check when a chamber soak temperature is set, so a missing sensor is caught up front rather than mid-print.

## Project layout

The workspace is made up of eight crates:

- **geo2d** — integer 2D geometry ("Clipper space", i64 nanometers at 1 mm = 1,000,000 units) with a Clipper2 wrapper for offset/boolean/stroke operations
- **mesh** — indexed triangle-mesh storage with STL (binary/ASCII) and 3MF import
- **gcode** — a dependency-free G-code string emitter for relative-extrusion moves, temperatures, fans, and G2/G3 arcs
- **config** — the three-tier TOML profile system with single-parent inheritance and embedded built-ins
- **printhost** — a blocking Moonraker/Klipper HTTP client for upload, print control, status, and the chamber-sensor preflight
- **engine** — the slicer core (slicing, planning, fill, and G-code emission), parallelized with rayon
- **cli** — the command-line front-end (binary `fable-slicer-cli`)
- **gui** — the primary desktop GUI (binary `fable-slicer`), built on eframe/egui with a wgpu 3D viewport

## License

Fable Slicer is licensed under the GNU Affero General Public License, version 3 or later (AGPL-3.0-or-later). See the [`LICENSE`](LICENSE) file for the full text.
