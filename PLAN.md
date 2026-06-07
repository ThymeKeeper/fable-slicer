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
| **Current milestone** | M0 — slicing core |
| **Last updated** | 2026-06-06 |
| **Builds / tests** | `cargo test` green (cube slices to 100 squares) |
| **Next action** | wire Clipper2 into `geo2d`, start M1 (walls + first g-code) |

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
- [ ] Slice a real-world STL (e.g. 3DBenchy) without panicking; eyeball the SVGs
- [ ] Golden-file test harness (snapshot SVG/loop output for a couple of fixtures)

### M1 — First printable cube
*Goal: get extruded plastic out of the machine.*

**Acceptance:** sliced cube prints on a real printer as a solid/hollow cube.

- [ ] Integrate **Clipper2** in `geo2d` (offset + boolean), adopt integer coords end-to-end
- [ ] Single outer wall via inward offset (`line_width/2`)
- [ ] Simple line/grid infill clipped to the wall interior
- [ ] Extrusion math (`E = length * line_width * layer_height / filament_area`)
- [ ] `gcode`: minimal AST + writer (G0/G1, set temp/fan, home, prime)
- [ ] Start/end g-code (hard-coded for one printer to begin with)
- [ ] CLI emits `.gcode`; verify in an external g-code previewer
- [ ] **First real print**

### M2 — Printable Benchy (the real quality bar starts here)
- [ ] Multiple walls (concentric offsets) + outer/inner ordering
- [ ] Top/bottom solid layers via boolean diff across N layers
- [ ] Sparse vs. solid region detection
- [ ] Skirt / brim / (basic) raft
- [ ] Retraction + travel moves; z-hop
- [ ] First-layer overrides (height, speed, flow, temp)
- [ ] `config`: tiered profile model (printer/filament/process) + override resolution (TOML/serde)
- [ ] **Benchy prints cleanly**

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
