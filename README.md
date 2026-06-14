<p align="center">
  <img width="2880" height="1713" alt="screenshot" src="https://github.com/user-attachments/assets/0a18f8ba-a109-4140-9574-11534120c39b" />
</p>

<img width="2880" height="1715" alt="screenshot2" src="https://github.com/user-attachments/assets/1884f89a-81d1-4818-9ffe-87b4d4575eb6" />

# Fable Slicer

![Status](https://img.shields.io/badge/status-work_in_progress-yellow.svg)

A from-scratch FDM slicer in Rust. GUI-first (egui + wgpu), Klipper-flavored
g-code out, and a deliberately small settings surface.

## Why another slicer

Nearly every mainstream slicer is one family tree: **Slic3r → PrusaSlicer →
Bambu Studio → OrcaSlicer**. That lineage is excellent — and also two decades
deep in inherited code, inherited UI, and above all inherited *settings*.
Hundreds of knobs survive not because users need them but because removing
them would break someone's muscle memory.

Fable Slicer starts from the ground up, and aims to stay minimal and lean. The concept: you provide a filament profile, a printer profile, and your strength preferences (walls, infill, layer height). Everything else — every speed, every temperature, every flow limit — is derived from those three, under the filament's melt ceiling. There is no print-speed slider. There is no temperature slider. The system chooses temperatures and speeds to maximize bead coalescence while avoiding the layer-to-layer temperature swings that print shrinkage rings into the surface.

## Heat control

The headline feature. The slicer models per-island heat load (deposited joules ÷ time × footprint) for every layer, pins per-layer targets, and serves them with three levers, planned entirely at slice time:

- a **nozzle-temperature schedule** — slew-limited M104 fades (≈1 °C/layer) derived from the hotend's measured thermal response, warming cold bulk for free and cooling hot zones inside the spool's printed range;
- **per-island slowdowns** where temperature runs out of authority;
- **park-and-wait dwells** — find a place to park the toolhead and wait for a layer to cool slightly when slowing the flow rate isnt enough.

The result: even heat across layer transitions (the banding killer), and good prints without much effort.

## What works today

- Own geometry kernel; arachne-style variable-width walls and a classic mode; exact skeletal trapezoidation (ported from CuraEngine — the one ported subsystem, attributed in-file); gap fill
- Top/bottom skins as first-class features (outer-wall pace, monotonic, proper `;TYPE:` labels), internal solid, five sparse patterns including gyroid
- Bridges, internal bridges, arc overhangs, grid + arc supports
- Seam strategies with real corner detection (a filleted corner is still a corner) and vertical seam-column tracking
- Arc fitting (G2/G3), vase mode, fuzzy skin, ironing, brick layering, half-height outer walls, elephant-foot / XY compensation, combed travels
- 3D preview with per-feature coloring, heat-load and nozzle-temperature maps, seam markers, layer scrubbing
- Klipper/Moonraker integration: send & print, live status, pause/resume, thermal profiling
- Slices a Benchy in roughly a third of a second

Daily-driven on a Sovol Zero (profiles included for Sovol Zero, Voron 2.4, and a generic machine). CLI (`fable-slicer-cli`) for scripted slicing.

## What doesn't exist yet

Multi-material, tree supports, multi-plate, paint-on anything, an installer, a settings search, most QoL you're used to. Expect rough edges and breaking profile changes. Issues and PR's are welcome.

## Building

```sh
cargo run --release -p gui    # the app (binary: fable-slicer)
cargo run --release -p cli -- model.stl   # fable-slicer-cli
```

Rust stable, Linux-first (X11/Wayland).

## License

[AGPL-3.0-or-later](LICENSE)

One subsystem is ported rather than original: the Arachne skeletal
trapezoidation (`engine/src/skeletal.rs`) derives from
[CuraEngine](https://github.com/Ultimaker/CuraEngine)'s implementation
(© UltiMaker, AGPL-3.0-or-higher) and carries that attribution in its file
header. Everything else in the engine is original work, and all third-party
crates are permissively licensed.
