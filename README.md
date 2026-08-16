<img width="2880" height="1800" alt="Screenshot from 2026-08-16 15-27-13" src="https://github.com/user-attachments/assets/46658e31-fcd1-4d1f-88c8-9180d5ada765" />


# Fable Slicer

Fable Slicer is a from-scratch FDM 3D-printing slicer written in Rust. It takes STL and 3MF models in and produces Klipper-flavored G-code out. The desktop GUI is the primary interface — a warm dark egui application with a custom wgpu 3D viewport for arranging models, slicing, previewing toolpaths, and sending jobs to a printer — and a command-line front-end is also available for scripted and headless use. Under both front-ends is a headless engine library that runs the full pipeline: mesh → layers → walls → infill → toolpaths → G-code.

## Features

### Slicing
- Mesh slicing by horizontal-plane intersection, parallelized with rayon, with triangle bucketing by z-span and integer/fixed-point XY coordinates
- Contour stitching with orientation by nesting depth (CCW outers, CW holes)
- Distinct first-layer height and layer height
- Multi-part models sliced per part on one shared layer grid: each part keeps its own walls, skins, and infill (full shells at part interfaces — no show-through between filaments), while overhang, bridge, and support decisions see the union of all parts, so a part resting on another is supported, not floating; overlapping volumes are owned by the earlier part

### Multi-material (toolchanger)
- Per-part filament assignment for multi-part 3MF models — the printer profile declares its tool count, each tool slot maps to a filament profile, and every part of the model picks a tool
- Per-slot spool colors: every tool slot in the GUI has its own color picker — the color describes the spool loaded in the slot (three slots can share one filament profile and differ only in color), overriding the profile's default and persisting with the slot; changing a slot's filament returns to the profile's own color. The whole loadout — slot filaments, spool colors, the blend palette, and the last process — is remembered **per printer**: machines don't share spools, so switching printers swaps in that machine's own setup instead of dragging one loadout across all of them
- **Blend palette**: create pseudo colors as weighted mixes of the loaded tools (three greys dither into a full monochrome ramp) and paint parts with them — realized by layer dithering (weighted error diffusion alternates whole layers between the tools, exact to ±1 layer over any stretch; parts sharing a blend band in lockstep, so they add no extra toolchanges). Visible faces show exactly one layer's filament — no averaging can happen there — so blends anchor their faces to the dominant tool: the topmost printed layer swaps with the nearest dominant layer below (ratio-neutral), and any other exposed face — a shelf on a stepped model, a floating underside, the bed face — big enough to be worth a dock trip (survives a 1.5-line-width erosion with 50 mm² of face left) is recolored in place; buried paths and part-to-part interfaces keep the dither exact. The model view shows the mixed color; the filament-colored preview shows the physical layer bands. Display colors too close to the viewport's near-black stage are nudged lighter just enough to stay visible — a black spool must not render as a hole — without touching the color anywhere it matters (profiles, blends, g-code)
- Blends are created by **picking a color, not dialing weights**: at every spool count the editor lays out the printable palette itself — one clickable swatch chip per distinct mixable color the blend band affords, sorted neutral-ramp-first then by hue family (dark→light), each chip one whole-layer recipe with a ring on the blend's current mix; when a palette is too rich to enumerate (many spools × fine layers) a free color picker takes over, snapping any color to the nearest achievable mix (constrained least squares in linear light), and the weight rows remain for fine-tuning
- Spool colors are typed as well as picked: each tool slot's hex field takes the color code straight off the spool label (`#RRGGBB`, enter to apply)
- On a toolchanger there is **no global filament selection**: the Filament card carries a tool-tab strip and edits each slot's own settings against that slot's own profile — per-tab dirty markers, and saving writes only the visible slot's changes to its own profile lineage, so one filament's values can never be saved over another's (single-tool printers keep the classic filament tier row)
- Every blend carries its own **sub-palette**: toggle which spools it draws from, and the surface follows the *chosen* count — narrow an eight-tool machine to two spools and you get the ramp back, three the triangle; the solver, chips, and weight rows all confine themselves to the selection
- The palette is **perception-bounded**: a process-tier *blend band* (default 0.8 mm) caps how tall a dither repeat may be and still fuse into one color, so mixes quantize to whole layers of the band ÷ layer-height cycle — the picker shows exactly that lattice of printable colors and refuses ratios that would stripe (one layer in ten is a 2 mm band, not a color); hand-dialed weights that exceed the band get a loud "will band" warning with the computed repeat height
- 3MF part names and extruder assignments read from Bambu/Orca `Metadata/model_settings.config`, so imported projects arrive pre-named and pre-assigned
- Within each layer, paths group by tool and consecutive layers serpentine through the tools, so a two-tool print pays one toolchange per layer
- Klipper toolchanger G-code (StealthChanger-style `T0`/`T1`/… macros, template configurable per printer): per-tool temperatures with preheat and per-tool first-layer drop, per-tool pressure advance, fan, flow limits, and filament diameter, with full state re-establishment after each swap — and no purge tower, since every tool has its own nozzle
- Standby temperature management for true multi-material jobs: a tool docked longer than the machine's threshold drops to its filament's standby temperature (auto: operating − 50 °C) instead of oozing and cooking at print temp, reheats a layer ahead of its next pickup, and the pickup swap confirms with a blocking wait; quick blend alternation never thermal-cycles
- Per-tool filament usage estimates and a toolchange time term in the print-time estimate

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
- Guided calibration towers: a single-wall teardrop printed as a seamless helix — the flow tower sweeps the extrusion multiplier with height (caliper the height where the wall reads exactly the line width), the pressure-advance tower sweeps PA (find the height where the single 90° corner is crispest); each tower holds the other parameter constant, so it doubles as that value's verification

### GUI
- Import STL and 3MF (a 3MF splits into one object per build item, preserving plate layout; an object keeps its named parts, each assignable to a tool)
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

# Multi-material on a toolchanger: one --filament per tool slot; a multi-part
# 3MF's parts print on their embedded extruder assignments
fable-slicer-cli figurine.3mf --filament pla-white --filament pla-grey --filament pla-black

# List available profiles and exit
fable-slicer-cli --list-profiles

# Dump per-layer toolpath SVGs while slicing
fable-slicer-cli model.stl --svg ./svg_out
```

Run the full test suite across all crates with `cargo test --workspace`.

## Profiles

Settings are organized into three tiers, each field owned by exactly one tier:

- **Printer** — machine geometry, motion limits, retraction, arc fitting, host connection, start/end G-code, and the toolchanger declaration (tool count, toolchange G-code template, seconds per change)
- **Filament** — material class, temperatures, cooling, flow, pressure advance, bridge settings, and display color
- **Process** — layer heights, walls, infill, supports, seams, surface features, and the speed/quality dial

On a toolchanger, one filament profile is loaded per tool slot. Shared heaters aggregate across the loaded filaments (hottest bed and chamber wish wins), and derived feature speeds respect the slowest filament's flow ceiling.

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

Both front-ends can talk to a Moonraker/Klipper host. The GUI offers a connection test, upload, upload-and-print, and a live-print card with a progress bar and pause/resume/cancel. The CLI exposes `--upload`, `--start-print`, and `--host` (falling back to the printer profile's host URL). Before sending, Fable Slicer runs pre-flight checks against the machine: the chamber-sensor check when a chamber soak is set, and — for multi-tool jobs — a tool check that every tool the slice uses actually exists on the connected Klipper (its extruders and `T<n>` macros), so slicing for the toolchanger and sending to the single-tool printer fails up front with a clear message instead of mid-print. The tool count itself is a declared printer-profile fact — the machine is never polled to configure anything, only to validate a send.

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
