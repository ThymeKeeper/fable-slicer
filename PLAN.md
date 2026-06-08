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
| **Current milestone** | M5/M6 — bed layout + brick layering landed (atop M4 supports/arc/bridge) |
| **Last updated** | 2026-06-08 |
| **Builds / tests** | `cargo test` green (33 tests). GUI verify = user screenshots (headless box) |
| **Next action** | auto-orient (lay-flat), or M5 (variable layers, gyroid, 3MF, G2/G3) |
| **Target printers** | Voron 2.4 = **350×350**, Sovol Zero = **152.4×152.4×152.5** (both confirmed). Klipper (relative E, PRINT_START). |

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
- [x] Solid infill: a perimeter loop around each solid region (a clean concentric bead along shell walls; absorbs thin bands so no lone strands), then straight-fill the interior. Sub-line-width slivers removed via morphological *open* + a minimum segment length.
- [x] Retraction on travels between extrusions; **z-hop** on travels that can't be combed (cross a void between islands)
- [x] Klipper-flavored output: relative extrusion (M83) + `--printer` presets (voron24 / sovol-zero)
- [x] Skirt (loops + gap, auto-clears any brim) and brim (loops, touching the part for adhesion) on the first layer; raft still TODO
- [x] First-layer height override (separate first-layer thickness); first-layer flow tuning still TODO
- [x] `config`: tiered profile model (printer/filament/process) + single-parent inheritance, TOML, built-in profiles, `--profile-dir`, `--list-profiles`
- [x] Printer profiles carry start/end g-code templates with `{placeholders}`; Voron/Sovol use `PRINT_START`/`PRINT_END` macros
- [ ] **Benchy prints cleanly** (needs hardware)

### M3 — Quality pass (what separates "prints" from "looks good")
- [x] Per-feature speeds (external perimeter slowed) + min-layer-time cooling: layers faster than `min_layer_time_s` slow to a floor speed (per-layer `speed_scale`)
- [x] Seam placement (nearest/rear · sharpest corner · random) — CLI `--seam` + GUI dropdown + GUI seam-highlight toggle
- [x] Travel ordering (nearest-neighbour) + **combing**: travels are planned once (`emit::plan_travels`) and stored on each layer, so g-code and the GUI preview share one source of truth (preview renders the *actual* combed routes, not naive straight lines). A travel that would cross a wall is rerouted via a per-layer visibility graph over the layer outline, routing around holes; a travel with no in-region route (between separate islands, across a void) retracts and **z-hops** over the gap. Benchy: travel 67m→17m, retractions 5587→~400 (all z-hopped).
- [x] Contour-resolution cleanup: merge sub-`max_resolution_mm` (0.05) mesh-facet noise after slicing — cleaner walls/preview, faster planning. Benchy g-code 236k→126k lines. (`dump_layer` example inspects a layer's raw contour roughness.)
- [ ] Gap fill between colliding offsets
- [x] Print-time estimate via trapezoidal motion simulation (acceleration + jerk-based junction look-ahead) + filament length/weight estimate; shown in GUI status + CLI
- [ ] Coasting / wipe

### M4 — Supports & bridging
- [x] Overhang detection: per-layer region not over the layer below within a printable cantilever (`support_overhang_angle_deg` from vertical ⇒ `h·tan(angle)`); thin slivers removed via morphological open
- [x] Normal **grid supports**: project overhangs downward, sparse-line fill (`PathKind::Support`) with XY clearance from the part; `--support none|grid|arc`, GUI picker + "support" preview category (`gen_overhang` fixture). **Z-gap** (`support_z_gap_layers`, default 1) leaves empty layers under the overhang for removal; **dense interface** (`support_interface_layers`, default 2) prints the top support layers solid for a smoother underside.
- [x] **Arc overhangs** (no-support option, McCulloch technique): flat interior overhangs are filled with self-supporting concentric arcs, printed slow as `PathKind::Bridge`; `--support arc` / GUI. Seeds at true concave corners (one per corner; falls back to spreading along a single straight supported edge), then McCulloch-style **farthest-point chaining** extends each fan from its frontier when it reaches `rmax` or stalls. Fans grow concentric rings in lockstep so they meet from several sides — bridges (supported on ≥2 sides) span further. Each fan owns the cells it fills; a ring is a continuous arc that stops at the region edge, anchor (supported) cells, and *other* fans' cells — so arcs stay inside the overhang (no bleeding into neighbouring fill), don't break on their own prior rings (no aliasing gaps), and meet cleanly where two fans touch (no overlap). Seeds anchor only on material held up by the **layer below**, so it auto-distinguishes a **bridge** (supported on ≥2 sides → seeds each side, fans meet) from a **cantilever** (1 side → seeds only there, arcs grow outward over air, McCulloch-style). Corner-seeded fans (convex corners from the polygon) + farthest-point chaining fill the region — rings sampled finer than a cell to avoid aliasing gaps, fans probe past covered strips, and each arc's **ends are handled by where they stop**: at the region boundary (wall) they snap right to it (binary-searched against the polygon) so they reach the edge; at a fan seam they overlap a small **tunable** amount (`arc_seam_overlap_mm`, default 0.1/fan) so neighbouring fans mesh without over-extruding the join. All arcs, no perimeter/patch (coverage tracked on a line-width grid). Verified on `gen_overhang` + coverage(≥85%)/containment unit tests. (v1: interior only — perimeters over overhangs still print normal; no per-feature cold/fan yet.)
- [ ] Support painting (manual enforce/block) — API-level
- [x] Bridge detection (in arc mode): each disjoint overhang **island** is decided on its own — a true bridge (supported ≥2 sides) narrower than `max_bridge_span_mm` (6mm) is filled with **straight lines across the shortest gap** (orientation = min max-span over candidate angles), anchored both ends, at bridge speed; wider gaps and cantilevers fall through to arcs. So a short gap bridges even next to a wide one on the same layer. (`gen_bridge`/`gen_overhang_suite` fixtures; 4 unit tests.) Per-feature bridge flow/fan and bridge detection in grid/none modes still TODO.

### M5 — Usability: GUI, profiles, advanced geometry
> **GUI pulled forward to now (user request, 2026-06-06).** Approach being scoped —
> framework `egui`; 2D-vs-3D preview TBD. Headless dev box can't render a window,
> so visual verification needs the user (or an xvfb screenshot harness).
- [x] GUI shell (`egui` + `wgpu`): load STL, pick profiles, edit settings, slice, export g-code; **3D model viewport** (orbit/zoom/pan, bed grid)
- [x] 3D toolpath preview: feature-colored **3D beads** (real line-width × layer-height, rounded/oval cross-section, rounded ends + corner-filling joint blobs, GPU-instanced) + travel moves, Model/Preview toggle, layer slider, per-category visibility toggles, and dimming of lower layers when scrubbing (layer/category/dim all in-shader from uniforms — no rebuild on scrub/toggle)
- [x] Profile system + starter library (printer/filament/process) — landed early; *management UI* still TODO
- [x] **Categorized settings panel** (Orca-style): settings grouped into collapsible sections (Quality, Walls & top/bottom, Infill, Speed, Support, Bed adhesion, Material & temperature, Cooling, Retraction, Machine) in a scroll area, with Slice/Export/preview pinned above. Exposes ~all `Settings` fields (was ~8). `max_arc_radius_mm` (rMax) and `bridge_speed_mm_s` are now real settings; accel/jerk live in Speed and are **emitted as Klipper motion limits** (`M204 S`, `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY`) right after PRINT_START, not just used for the time estimate. *Scales for the many more knobs to come; per-setting profile-override indicators + a Marlin g-code flavor (M205) still TODO.*
- [~] **Multi-object scene / bed layout** (steps 1–2 done): import multiple STLs, duplicate/delete, object list with selection (selected object highlighted in 3D), auto grid-arrange, slice & render all objects together. Each object = shared `Arc<Mesh>` + an editable placement (Euler rotation, uniform scale, bed-plane position) that bakes to a `mesh::Transform` always resting on z=0; the combined scene is baked to one mesh in bed coords and the slicer's auto-centering is disabled (`Settings::auto_center_on_bed`). **Transforms:** panel controls (move/rotate XYZ/scale + center/reset) **and** viewport interaction — left-click to select, left-drag an object to move it on the bed (ray-pick + z=0 plane), drag empty space to orbit. **Next:** step 3 auto-orient (lay-flat onto a face); viewport rotate/scale gizmos still panel-only.
- [ ] Variable / adaptive layer height
- [x] Infill patterns: lines / grid / concentric for both sparse and solid (GUI + CLI picker, `sparse_infill`/`solid_infill` profile keys); gyroid still TODO
- [x] **G2/G3 arc fitting** (opt-in, `arc_fitting` + `arc_tolerance_mm`): the emitter folds runs of toolpath points that lie on a circle into a single G2/G3 (smaller g-code, smoother motion; pairs with arc overhangs). Greedy circle fit per run; a run only qualifies if every chord *hugs* the arc (sagitta ≤ tolerance), so concyclic polygon corners (e.g. a square) stay straight — only genuine curves/rounded-corners convert. Center refit on endpoints+midpoint for accurate I/J; arc-length E. GUI toggle + tol (Quality), CLI `--arc-fitting`. Needs firmware arc support (Klipper `[gcode_arcs]`). Verified: holeplate circles → G2/G3, cube walls stay G1 (only rounded skirt corners arc); unit test (circle yes, square no).
- [ ] 3MF load (zip + XML): multi-object, transforms, embedded settings

### M6 — Advanced / parity
- [x] **Brick layering** (opt-in, `brick_layers` + `brick_flow`): odd-indexed perimeters are lifted half a layer height (outer wall = index 0 stays put) so adjacent wall rings interlock like masonry — staggered inter-layer seams resist delamination. The lifted rings get a flow bump (default 1.05) to fuse into the valley; first/last layers are a flat transition/clamp. Per-path `z_offset_mm`/`flow` on `ToolPath`, honored by the emitter (Z + extrusion) and the 3D preview (beads shown lifted + fatter). **Brick-aware motion:** the planner prints the on-plane (low) phase fully before the lifted (high) phase (`order_layers` groups by Z phase), and any travel into/out of a lifted perimeter is forced to **retract + Z-hop** clear (≥ a full bead) so the nozzle never drags through a bead at the other Z — no more per-ring Z-bobbing. CLI `--brick`.
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
- **Profiles: Prusa-style named profiles + single-parent inheritance** (not Cura's
  computed-setting graph). Three tiers (printer/filament/process) of all-`Option`
  fields, merged child-over-parent, resolved to a flat `Settings`; unset fields
  fall back to code defaults. Start/end g-code are templates with `{placeholders}`
  on the printer profile — Klipper printers call `PRINT_START`/`PRINT_END` macros.
- **GUI: egui + wgpu (eframe), own offscreen render pass.** Renders the scene to a
  color+depth texture shown via egui's native-texture path (gives a real depth
  buffer for 3D). **eframe pinned to `=0.34.1`** — 0.34.3 requires `egui_glow 0.34.3`,
  which was never published. wgpu is 29.x. GUI can't be rendered on this headless
  box; verification is by the user's screenshots.

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
