<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/a2f4ff3c-3402-451b-8259-bbc7f6a9b606" />

<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/24ff77b1-d16d-428f-8704-418b6e8d649e" />

# Fable Slicer

![Status](https://img.shields.io/badge/status-work_in_progress-yellow.svg)

A from-scratch FDM slicer written in Rust. GUI-first (egui + wgpu), Klipper-flavored
g-code out, and a deliberately small settings surface — most of what other slicers
expose as a knob, Fable derives.

## Why another slicer

Almost every mainstream slicer shares one family tree: **Slic3r → PrusaSlicer →
Bambu Studio → OrcaSlicer**. That lineage is excellent, and also two decades deep
in inherited code, inherited UI, and — above all — inherited *settings*. Hundreds
of knobs survive not because anyone needs them but because removing one would
break someone's muscle memory.

Fable starts over and tries to stay small. The bet: a slicer should ask what
you're making something out of, what you're making it on, and how strong or fine
you want it — then work out the rest itself.

## The idea

You configure three things:

- **Filament** — a packaging card. Its material class (PLA, PETG, ABS/ASA, TPU…),
  a nozzle temperature, and its volumetric melt ceiling.
- **Printer** — a datasheet. Build volume, rated speed and acceleration, and the
  hardware it actually has (part / aux / chamber fans, a chamber sensor).
- **Process** — your intent. Wall count, infill density and pattern, layer height,
  and a single finish↔speed dial.

Everything downstream — every per-feature speed, the flow per move, the first-layer
temperature, fan duties — is **derived** from those three, and clamped under the
filament's volumetric melt ceiling so the hotend is never asked for more plastic
than it can melt. There are no per-feature speed sliders to balance; the nozzle
temperature is the one thermal number you set by hand, and the rest follows.

## Heat control

Cooling a layer unevenly is what prints shrinkage banding into a surface. Fable
models a **per-layer heat load** — the power a layer deposits spread over the area
it covers (mW/mm²) — and, when heat control is on, holds that load under a ceiling
by **pacing the print speed**: layers that would run hot are slowed, with a
gradient limit so the change eases in over several layers instead of snapping. You
give it one switch and a budget ("spend up to N% more print time smoothing heat")
and it plans the rest at slice time. The payoff is even heat across layer
transitions and cleaner walls, with no calibration ritual. The preview can color
every layer by that heat load so you can see where it spends the time.

## What works today

- A homegrown 2D geometry kernel; **Arachne-style variable-width walls** plus a
  classic fixed-width mode; exact skeletal trapezoidation (ported from CuraEngine —
  the one ported subsystem, attributed in-file); gap fill
- Top/bottom skins as first-class features (outer-wall pace, monotonic ordering,
  proper `;TYPE:` labels), internal solid fill, and **six infill patterns**: lines,
  aligned lines, grid, triangles, concentric, and gyroid
- Bridges, internal bridges, arc-fitted overhangs, and **grid + arc supports** —
  grid columns are wrapped in a perimeter so thin sections stay self-supporting
- Seam strategies with real corner detection (a filleted corner is still a corner)
  and vertical seam-column tracking
- Arc fitting (G2/G3), spiral vase, fuzzy skin, ironing, brick layering, half-height
  outer walls, elephant-foot / XY compensation, combed travels
- A **wgpu 3D preview** — multisampled, per-feature coloring, layer-time and
  heat-load maps, seam markers, a vertical layer scrubber, and multi-bed plating
- **Klipper / Moonraker** integration: send & print, live status, pause/resume, and
  a chamber pre-soak gated on the filament
- Slices a Benchy in roughly a third of a second

Daily-driven on a Sovol Zero. Bundled profiles cover the Sovol Zero, Voron 2.4, and
a generic machine; PLA (plus a high-flow PLA), PETG, ABS, ASA, and Polymaker PC
filaments; and draft / standard / fine processes. A CLI (`fable-slicer-cli`) handles
scripted slicing.

## What doesn't exist yet

Multi-material, tree supports, paint-on supports and seams, an installer, settings
search — most of the quality-of-life you're used to. Expect rough edges and
breaking profile changes. Issues and PRs welcome.

## Building

```sh
cargo run --release -p gui                 # the app (binary: fable-slicer)
cargo run --release -p cli -- model.stl    # the CLI (fable-slicer-cli)
```

Rust stable, Linux-first (X11 / Wayland via wgpu).

## License

[AGPL-3.0-or-later](LICENSE)

One subsystem is ported rather than original: the Arachne skeletal trapezoidation in
`engine/src/skeletal.rs` derives from
[CuraEngine](https://github.com/Ultimaker/CuraEngine)'s implementation
(© UltiMaker, AGPL-3.0-or-higher) and carries that attribution in its file header.
Everything else in the engine is original work, and all third-party crates are
permissively licensed.
