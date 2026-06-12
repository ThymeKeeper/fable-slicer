# Fable Slicer

A from-scratch FDM slicer in Rust. GUI-first (egui + wgpu), Klipper-flavored
g-code out, and a deliberately small settings surface.

**Status: work in progress** — it prints real parts on a real printer daily,
and it is nowhere near finished.

**Full disclosure:** this project is vibe-coded with Claude (Anthropic's
*Fable* model — the slicer is named after it). A human picks the direction,
argues with the results, files the bug reports, and prints the Benchies;
Claude writes essentially all of the code. If that offends your sensibilities,
this is not your slicer. If you're curious what that workflow can build, read
on.

## Why another slicer

Nearly every mainstream slicer is one family tree: **Slic3r → PrusaSlicer →
Bambu Studio → OrcaSlicer**. That lineage is excellent — and it is also two
decades deep in inherited code, inherited UI, and above all inherited
*settings*. Hundreds of knobs survive not because users need them but because
removing them would break someone's muscle memory.

Fable Slicer is an attempt at a genuine alternative: a clean-room rewrite that
asks what a slicer looks like if you start today. The founding idea is that
**most slicer settings shouldn't exist**:

- the **printer** profile is a datasheet — rated speed, acceleration, build
  volume, the hardware it has;
- the **filament** profile is the packaging card — the material class and the
  temperature range printed on the box, plus measured calibration values;
- the **process** profile is geometry intent — layer height, walls, infill.

Everything else — every speed, every temperature, every flow limit — is
*derived* from those three, live, under the filament's melt ceiling. There is
no print-speed slider. There is no temperature slider. When the system
intervenes, the g-code header says exactly what it did and why.

## Heat control

The headline feature. The slicer models per-island heat load (deposited joules
÷ time × footprint) for every layer, pins per-layer targets, and serves them
with three levers, planned entirely at slice time:

- a **nozzle-temperature schedule** — slew-limited M104 fades (≈1 °C/layer)
  derived from the hotend's *measured* thermal response, warming cold bulk
  for free and cooling hot zones inside the spool's printed range;
- **per-island slowdowns** where temperature runs out of authority;
- **park-and-wait dwells** — parked over sparse infill so ooze lands where the
  next layers bury it — for tiny islands that run out of path to slow.

The result: even heat across layer transitions (the banding killer), a
chimney that doesn't melt, and a report of every intervention in the g-code
header. One switch, on by default; one preference (how much extra print time
smoothing may spend). The hotend thermal profiler runs over Moonraker with one
click and saves the measured rates into the printer profile.

## What works today

- Own geometry kernel; arachne-style variable-width walls *and* a classic
  mode; exact skeletal trapezoidation; gap fill
- Top/bottom skins as first-class features (outer-wall pace, monotonic,
  proper `;TYPE:` labels), internal solid, five sparse patterns including
  gyroid
- Bridges, internal bridges, arc overhangs, grid + arc supports
- Seam strategies with real corner detection (a filleted corner is still a
  corner) and vertical seam-column tracking
- Arc fitting (G2/G3), vase mode, fuzzy skin, ironing, brick layering,
  half-height outer walls, elephant-foot / XY compensation, combed travels
- 3D preview with per-feature coloring, heat-load and nozzle-temperature
  maps, seam markers, layer scrubbing
- Klipper/Moonraker integration: send & print, live status, pause/resume,
  thermal profiling
- Slices a Benchy in roughly a third of a second

Daily-driven on a Sovol Zero (profiles included for Sovol Zero, Voron 2.4,
and a generic machine). CLI (`fable-slicer-cli`) for scripted slicing.

## What doesn't exist yet

Multi-material, tree supports, multi-plate, paint-on anything, an installer,
a settings search, most QoL you're used to. Expect rough edges and breaking
profile changes. Issues are welcome; just know the maintainer will probably
hand your bug report to the machine that wrote the bug.

## Build & run

```sh
cargo run --release -p gui    # the app (binary: fable-slicer)
cargo run --release -p cli -- model.stl --printer sovol-zero --filament pla --process standard
```

Rust stable, Linux-first (X11/Wayland); wgpu with a GL fallback.

## License

AGPL-3.0-or-later.
