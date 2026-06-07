# Slicer — Project Plan & Progress Tracker

A from-scratch FDM slicer in Rust. This file is the living roadmap: update the
checkboxes and the status line as work lands. Architecture detail lives in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

- **Goal:** ship a real tool people actually print with. Favor correctness,
  reliability, and the boring quality features over novelty.
- **Independence:** clean reimplementation, not a fork. GPL/AGPL is acceptable;
  read CuraEngine/PrusaSlicer for reference and port *algorithms*, never code.
- **Language/stack:** Rust workspace, `rayon` for per-layer parallelism, Clipper2
  for 2D polygon ops, `egui`/`wgpu` for the eventual GUI.

---

## Status

| | |
|---|---|
| **Current milestone** | M2 — surfaces + retraction (in progress) |
| **Last updated** | 2026-06-06 |
| **Builds / tests** | `cargo test` green (14 tests); cube watertight (solid top/bottom), centered, Klipper output |
| **Next action** | skirt/brim + first-layer height, then the real TOML profile system |
| **Target printers** | Voron 2.4 + Sovol Zero (both Klipper → relative E). Bed sizes need confirming. |

Legend: `[x]` done · `[~]` in progress · `[ ]` not started

---

## Milestones

### M0 — Slicing core (mesh → layer polygons) — **in progress**
*Goal: prove the geometric foundation. No printing yet; verify visually via SVG.*

**Acceptance:** slicing a 20mm cube at 0.2mm yields 100 layers, each a single
~400mm² CCW square; per-layer SVGs render a clean outline.

- [x] Cargo workspace + crate skeleton (`geo2d`, `mesh`, `engine`, `gcode`, `config`, `cli`)
- [x] `geo2d`: integer Point/Contour/Polygons, area, winding, point-in-polygon, AABB
- [x] `mesh`: indexed mesh, STL load (binary + ASCII auto-detect), ASCII STL write, cube primitive
- [x] `engine`: plane/triangle intersection, undirected segment stitching, nesting-based winding
- [x] `cli`: STL → slice → per-layer SVG
- [x] Cube acceptance test + SVG smoke output
- [ ] Robustness pass: degenerate triangles, coplanar facets, open edges (don't panic; best-effort loops)
- [ ] Per-layer parallelism with `rayon`
- [x] Slice a real-world STL (3DBenchy, 225k tris) without panicking — 240 layers in ~0.4s
- [ ] Golden-file test harness (snapshot SVG/loop output for a couple of fixtures)

### M1 — First printable cube
*Goal: get extruded plastic out of the machine.*

**Acceptance:** sliced cube prints on a real printer as a solid/hollow cube.

- [x] Integrate **Clipper2** in `geo2d` (offset via `One` scaler, integer coords) — boolean ops deferred to M2
- [x] Walls via inward offset (`line_width/2`, then concentric) — configurable `--walls`
- [x] Simple line infill clipped to the wall interior (even-odd scanline, alternating 45°/135°)
- [x] Extrusion math (`E = length * line_width * layer_height / filament_area`)
- [x] `gcode`: `GcodeBuilder` writer (G0/G1, temps, fan, home; absolute E)
- [x] Start/end g-code (generic Marlin placeholder)
- [x] CLI emits `.gcode` (cube verified; Benchy 240 layers / 190k lines in ~2s)
- [ ] **Center model on bed** — Benchy is modeled around the origin (negative coords); needed before non-trivial models print
- [ ] **First real print** (needs the target printer's start/end g-code + bed size)

### M2 — Printable Benchy (the real quality bar starts here)
- [x] Multiple walls (concentric offsets) — landed in M1; outer/inner *ordering* tuning still TODO
- [x] Auto-center / place model on the bed (handles models with negative coordinates)
- [x] Top/bottom solid layers via boolean diff across N layers (`top_layers`/`bottom_layers`)
- [x] Sparse vs. solid region detection (solid shells dense-filled, core sparse)
- [x] Retraction on travels between extrusions (z-hop still TODO)
- [x] Klipper-flavored output: relative extrusion (M83) + `--printer` presets (voron24 / sovol-zero)
- [ ] Skirt / brim / (basic) raft
- [ ] First-layer overrides (height, flow; speed already slowed)
- [ ] `config`: tiered profile model (printer/filament/process) + override resolution (TOML/serde) — still a flat struct + presets
- [ ] **Benchy prints cleanly** (needs hardware)

### M3 — Quality pass (what separates "prints" from "looks good")
- [ ] Per-feature speeds; min-layer-time cooling slowdown
- [ ] Seam placement (aligned / random / sharpest corner)
- [ ] Combing (travel inside the part to avoid stringing)
- [ ] Gap fill between colliding offsets
- [ ] Accurate time estimate via trapezoidal motion simulation
- [ ] Coasting / wipe

### M4 — Supports & bridging
- [ ] Overhang detection (per-layer unsupported regions)
- [ ] Normal grid supports + interface layers
- [ ] Support painting (manual enforce/block) — API-level
- [ ] Bridge detection; bridge flow/speed/fan; bridge line orientation along shortest span

### M5 — Usability: GUI, profiles, advanced geometry
- [ ] GUI shell (`egui` + `wgpu`): load, arrange, slice, preview
- [ ] G-code preview: feature-colored paths + layer slider
- [ ] Profile management UI; ship a starter profile library (the real moat)
- [ ] Variable / adaptive layer height
- [ ] Gyroid + more infill patterns
- [ ] G2/G3 arc fitting
- [ ] 3MF load (zip + XML): multi-object, transforms, embedded settings

### M6 — Advanced / parity
- [ ] Tree / organic supports
- [ ] Arachne-style variable-width walls
- [ ] Multi-material / multi-color (sequencing, prime tower)
- [ ] Ironing, fuzzy skin, scarf seams
- [ ] Network printing + monitoring

---

## Cross-cutting workstreams

**Testing & correctness**
- [ ] Golden-file snapshots for slices and (later) g-code; fail on diff
- [ ] Property tests: sliced area ≈ cross-section; loops closed; winding consistent
- [ ] Determinism: no `HashMap` iteration order in engine output (use sorted / `BTreeMap` / `IndexMap`)
- [ ] Corpus of nasty meshes (non-manifold, self-intersecting, open) the slicer must survive

**Performance** (defer until correct)
- [ ] Bucket triangles by z-span (avoid O(layers × triangles))
- [ ] `rayon` across layers; measure on a large model
- [ ] Avoid needless allocation in the hot path

**Robustness / mesh health**
- [ ] Tolerant slicing (union overlapping contours per layer) over upfront repair
- [ ] Targeted repair only where prints fail (hole fill, normal orientation)

---

## Decision log

- **Clean reimplementation, AGPL output.** Read CuraEngine (cleanest engine) and
  PrusaSlicer for reference; port algorithms, not code. The result is AGPL — an
  accepted trade for not maintaining a fork. Revisit only if a permissive license
  becomes a requirement (would forbid porting GPL code + using CGAL).
- **Clipper2 for 2D polygon ops.** Boost-licensed, robust, industry standard.
  Writing our own offsetting is a separate multi-month project we won't take on.
- **Integer ("Clipper-space") coordinates, nm resolution.** Exact shared vertices;
  no FP drift in booleans/stitching. Convert to mm only at I/O.
- **Topology-free stitching via snapped integer endpoints.** Points computed from
  adjacent triangles are bit-identical, so loops stitch exactly without an
  epsilon match or half-edge structure (kept simple for M0).
- **Brand-neutral crate names** (`engine`, not `slicer-core`); dir + binary are
  placeholders. Renaming the project is cheap. *(Pick the real name later.)*
- **Tolerant slicing over CGAL repair.** Avoid heavy C++/templated FFI; make the
  slicer survive imperfect meshes instead of perfecting them first.

## Open questions
- [ ] Real project name (current `slicer` is a placeholder).
- [ ] First target printer/firmware for M1 g-code (Marlin vs Klipper start/end).
- [ ] Cura-style computed settings vs. Prusa-style named-profile inheritance — start Prusa-style.

## Reference reading map (port concepts, not code)
- **CuraEngine:** `Slicer`/`SlicerLayer` (slicing + stitching), `WallToolPaths` →
  `SkeletalTrapezoidation` (Arachne), `infill/`, `LayerPlan` + `PathOrderOptimizer`
  + `comb/`, `TreeSupport*`, `TimeEstimateCalculator`, `FffGcodeWriter`.
- **PrusaSlicer:** `SeamPlacer` (seam hiding), `Geometry/ArcWelder` (arc fitting),
  organic supports, `BridgeDetector`.
- **Papers:** Arachne (Kuipers et al., 2020); Clipper2 docs; gyroid level-set.
