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
| **Current milestone** | M5/M6 — perf pass (rayon + z-buckets, ~4× faster), gyroid/triangles, monotonic solid, gap fill, fuzzy skin, ironing, spiral vase, elephant-foot/XY comp, per-feature speeds, fan/bridge control, pressure advance, M73 |
| **Last updated** | 2026-06-10 |
| **Builds / tests** | `cargo test --workspace` green (76 tests). GUI verify = user screenshots (headless box) |
| **Next action** | 3MF, auto-orient, variable layers. A real test print would beat all of it. (Arachne graph walk **done** 2026-06-10 — `engine/src/skeletal.rs`, both defect layers fixed, see [docs/ARACHNE-GRAPH-WALK.md](docs/ARACHNE-GRAPH-WALK.md)) |
| **Target printers** | Voron 2.4 = **350×350**, Sovol Zero = **152.4×152.4×152.5** (both confirmed). Klipper (relative E, PRINT_START). |
| **Perf (Benchy 225k tris)** | load 47ms · slice 29ms · plan ~190ms · g-code 66ms ≈ **0.33s** total (`cargo run --release -p engine --example bench`) |

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
- [x] Per-layer parallelism with `rayon` — slicing, wall/infill planning, travel combing, min-layer-time all layer-parallel
- [x] Triangle **z-bucketing**: each layer visits only triangles whose span crosses it (banded buckets cap memory on tall spans); coincident-vertex nudge via sorted unique-z table. Benchy slice 0.8s → **29ms**
- [x] Slice a real-world STL (3DBenchy, 225k tris) without panicking — full pipeline now ~0.33s (`engine --example bench`)
- [ ] Golden-file test harness (snapshot SVG/loop output for a couple of fixtures)

### M1 — First printable cube
*Goal: get extruded plastic out of the machine.*

**Acceptance:** sliced cube prints on a real printer as a solid/hollow cube.

- [x] Integrate **Clipper2** in `geo2d` (offset via `One` scaler, integer coords) — boolean ops deferred to M2
- [x] Walls via inward offset (`line_width/2`, then concentric) — configurable `--walls`
- [x] Simple line infill clipped to the wall interior (even-odd scanline, alternating 45°/135°)
- [x] Extrusion math — **stadium bead model** (2026-06-09): `E = length × bead_area / filament_area` with `bead_area = h·(w−h) + π·h²/4` (the physical cross-section; was a w×h rectangle that over-fed ~9.5% at defaults). Adjacent beads are *placed* at the stadium spacing `w − h·(1−π/4)` so shoulders overlap and fill the inter-bead cusps — walls, solid fill, sparse density, supports, and brim all use it; outer wall stays at lw/2 (dimensions unchanged)
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
- [x] **Auto/pin "priority modes"** (2026-06-09): derivable settings (line width ← nozzle ×112.5%, outer-wall/solid/support/gap speeds ← print speed) show a weak `auto` badge and **recompute live** while unpinned — drag print speed and its dependents visibly follow. Dragging a dependent pins it (badge becomes ⟲ to revert); pin state loads from profile provenance (field set in the chain = pinned) and **saves never serialize unpinned auto values** (they're derived, not chosen — also excluded from the dirty `*`). One derivation source: `config::derived_*`, shared by `resolve()` and the GUI. `standard` no longer pins line width (auto gives the same 0.45); `draft`'s 0.5 stays deliberately pinned
- [x] **Per-feature speeds as real settings** (`external_perimeter`/`solid`/`support`/`gap_fill` speeds; derived from the machine's print speed when a profile leaves them unset — external 50%, solid 80%, support 90%, gap 40% capped at 40) + min-layer-time cooling: layers faster than `min_layer_time_s` slow to a floor speed (per-layer `speed_scale`)
- [x] Seam placement (nearest/rear · sharpest corner · random) — CLI `--seam` + GUI dropdown + GUI seam-highlight toggle
- [x] Travel ordering (nearest-neighbour) + **combing**: travels are planned once (`emit::plan_travels`) and stored on each layer, so g-code and the GUI preview share one source of truth (preview renders the *actual* combed routes, not naive straight lines). A travel that would cross a wall is rerouted via a per-layer visibility graph over the layer outline, routing around holes; a travel with no in-region route (between separate islands, across a void) retracts and **z-hops** over the gap. Benchy: travel 67m→17m, retractions 5587→~400 (all z-hopped).
- [x] Contour-resolution cleanup: merge sub-`max_resolution_mm` (0.05) mesh-facet noise after slicing — cleaner walls/preview, faster planning. Benchy g-code 236k→126k lines. (`dump_layer` example inspects a layer's raw contour roughness.)
- [x] **Gap fill** between colliding offsets (`gap_fill`, default on): per wall, the strip where material remains at the bead's outer edge but the wall didn't fit (plus open-dropped infill slivers) becomes single width-matched strokes along the sliver's principal axis (PCA), at `gap_fill_speed`. Hair-thin offset noise is filtered (morphological open + width ≥ 0.3·lw). Benchy's thin hull gains ~0.27m of previously-missing material
- [x] **Feature annotations** (2026-06-10): `;TYPE:` comments per feature block in the g-code (Outer wall / Inner wall / Overhang wall / Solid infill / Sparse infill / Gap infill / Ironing / Support / Bridge / Skirt — the names Mainsail/Fluidd/Orca's viewer already colour-code), so printer UIs show features and analysis tools can attribute extrusion per feature. Born from the OrcaSlicer g-code comparison, where their `;TYPE:`s made their side of the analysis easy and ours guesswork
- [x] **Per-feature acceleration** (2026-06-10): `M204` follows the feature being printed — `outer_wall_accel` (auto = accel/2, ≥500) for the visible wall, `first_layer_accel` (auto = min(1000, accel)) for adhesion, the main `acceleration` for everything else (inner walls, fills, support, travel). Emitted before each feature's lead-in travel; the trapezoidal time estimator uses the same per-path accel so estimates track what the printer is told. Printer-tier profile fields + GUI auto/pin sliders under Machine & motion. voron24 raised to `acceleration = 10000` with the outer wall pinned at the old 3000 — interior/travel speed-up with zero visible-surface change (Klipper still clamps to printer.cfg `max_accel`; comment says to keep them in agreement). Benchy estimate 36m13s → **32m36s** (−10%), within 41s of Orca's 31m55s despite our 15 mm³/s flow cap vs their 21 and walls at 75–150 vs their 200–300 mm/s
- [x] **Overhang wall slowdown** (2026-06-10, `plan.rs slow_overhanging_walls`): wall stretches whose centerline lies past the previous layer's outline (>½ bead unsupported — Orca's 50% bucket) become `PathKind::OverhangWall`: printed at `overhang_speed` (auto = bridge speed; same physics, beads onto air) with bridge-grade fan, at outer-wall acceleration. Loops split into consecutive open pieces at classification borders (no travel between), runs under 2·lw merged into their neighbour so the speed doesn't chatter; detection is a per-layer `difference(layer, prev⊕0.05)` that is empty (free) on most layers; vase mode skips (the spiral loop must stay whole). Hot-amber in the GUI preview + CLI SVG; `;TYPE:Overhang wall` in g-code. Benchy: 105 overhang stretches, 465 mm — the bow/stern undersides now print at 15 mm/s with full fan instead of 75–150
- [x] **Internal bridges** (2026-06-10): the first solid layer over sparse infill prints as `PathKind::InternalBridge` — lines oriented **perpendicular to the sparse lines below** (each free span = one line spacing), extended half a bead into the surrounding solid to anchor, bridge-grade fan, flow 1.0 (anchored every cell; bridge under-extrusion would pinhole the surface). Speed is physics-derived, not another knob: free span = spacing/density, so `bridge_speed × (max_bridge_span / span)`, clamped to [bridge, solid] — 33 mm/s at defaults vs 15 for free-air bridges. Region = `solid[i] ∩ (inner[i−1] − solid[i−1])` from a precomputed per-layer solid split, morphologically opened; the solid boundary loop still rings the whole region (anchor rim), and the remaining solid fill excludes the bridged area. `;TYPE:Internal Bridge` (Orca's name), deep-teal in preview/SVG. Benchy: 168 bridge blocks; voron24 +1m06s — the cost of tops that don't sag into 15% infill
- [x] **Sovol Zero profile matched to Orca's high-speed numbers** (2026-06-10, pinned by a config test): interior accel + travel 40000 / outer wall 10000 / first layer auto→1000 (all = Orca's measured M204s), inner walls 400 mm/s, first layer 55, travel 1000, SCV 5; pair with `pla-hf` (30 mm³/s ≥ Orca's 21 — the melt ceiling is what actually limits speed, and the clamp reports loudly). Known divergence, documented in the profile: our external-wall auto stays at 50% of print speed (quality-first) where Orca asks 350 — pin "external mm/s" in the GUI for Orca's number. Benchy: **22m26s** (sovol+pla-hf) / 33m42s (voron24+pla), vs 36m13s at the session start
- [x] **Junk-solid filter** (2026-06-10, `plan.rs rebalance_solid_sparse`): the top/bottom coverage booleans shed solid islands too small (< ~2×2 beads) or nowhere wider than one line width (Cura's skin-removal-width default) — they printed as micro hairpin loops or nothing at all while still stealing the area from fill. Such islands are **demoted into the sparse region** (the space stays accounted for); the existing sparse→solid promotion (<10mm²) gets the same junk floor so demoted crumbs can't bounce back. Plus: solid straight-fill rows shorter than 1.5·lw are dropped (the solid boundary loop already covers the rim — each dab cost a travel for redundant material; sparse/support/bridge keep the 0.5mm floor), and pinched-off solid boundary loops under 4·lw perimeter are skipped. Benchy: sub-0.7mm solid paths 233 → 50, longer buckets and estimates unchanged (−0.4% solid length = pure overhead)
- [x] **Infill⇄wall overlap** (`infill_overlap`, default 25% of line width): sparse/solid fill (and the solid boundary loop) push into the innermost wall bead so they bond — was exactly zero squish before
- [x] **Monotonic solid fill** (`monotonic_solid`, default on): solid lines print in one strict boustrophedon sweep per island (`ToolPath::group` blocks survive travel ordering intact), so top surfaces get an even sheen
- [x] **Volumetric flow clamp** (`max_volumetric_speed_mm3_s`, filament tier; pla 15 / petg 11 / new `pla-hf` 30; 0 = off): every feed is clamped so `width × height × speed × flow` never exceeds the filament's melt ceiling — one function (`feed_for`) feeds g-code, the time estimate, and min-layer-time, so they always agree. **Loud, never silent**: the g-code header, CLI, and GUI status all report exactly what got slowed (`flow-limited: infill 250 → 167 mm/s`). Sovol Zero at 250 mm/s + generic PLA clamps (the announcement *is* the calibration prompt); pair it with `pla-hf` instead
- [x] Print-time estimate via trapezoidal motion simulation (acceleration + jerk-based junction look-ahead) + filament length/weight estimate (now honors per-path flow / bridge flow / extrusion multiplier); shown in GUI status + CLI
- [x] **M73 progress** (time-based percent + minutes remaining per layer) and metadata header comments (estimated time, filament mm/g, layer count)
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
- [x] Profile system + starter library (printer/filament/process) — landed early
- [x] **User profiles + management UI**: per-tier user profiles (the Prusa/Orca model) stored as minimal TOML diffs with `inherits = "<base>"` in the platform config dir (`~/.config/slicer/profiles/{printer,filament,process}/`), auto-loaded by GUI *and* CLI. GUI shows a per-tier `*` when panel edits diverge from the selected profile and routes each edited field to its owning tier on save (print/travel/first-layer speed → printer, since that tier wins resolve precedence). Save = diff vs. the resolved baseline + inherit the selection; overwriting a user profile merges the new diff over its stored fields and keeps its parent. Built-ins are read-only (save under their name is refused); delete is user-profiles-only with a confirm dialog. Saving one tier preserves un-saved edits in the others (baseline-only refresh)
- [x] **Categorized settings panel** (Orca-style): settings grouped into collapsible sections (Quality, Walls & top/bottom, Infill, Speed, Support, Bed adhesion, Material & temperature, Cooling, Retraction, Machine) in a scroll area, with Slice/Export/preview pinned above. Exposes ~all `Settings` fields (was ~8). `max_arc_radius_mm` (rMax) and `bridge_speed_mm_s` are now real settings; accel/jerk live in Speed and are **emitted as Klipper motion limits** (`M204 S`, `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY`) right after PRINT_START, not just used for the time estimate. *Scales for the many more knobs to come; per-setting profile-override indicators + a Marlin g-code flavor (M205) still TODO.*
- [~] **Multi-object scene / bed layout** (steps 1–2 done): import multiple STLs, duplicate/delete, object list with selection (selected object highlighted in 3D), auto grid-arrange, slice & render all objects together. Each object = shared `Arc<Mesh>` + an editable placement (Euler rotation, uniform scale, bed-plane position) that bakes to a `mesh::Transform` always resting on z=0; the combined scene is baked to one mesh in bed coords and the slicer's auto-centering is disabled (`Settings::auto_center_on_bed`). **Transforms:** panel controls (move/rotate XYZ/scale + center/reset) **and** viewport interaction — left-click to select, left-drag an object to move it on the bed (ray-pick + z=0 plane), drag empty space to orbit. **Next:** step 3 auto-orient (lay-flat onto a face); viewport rotate/scale gizmos still panel-only.
- [ ] Variable / adaptive layer height
- [x] Infill patterns: lines / grid / **triangles** / concentric / **gyroid** for both sparse and solid (GUI + CLI picker, `sparse_infill`/`solid_infill` profile keys). Gyroid = marching squares on the level set per layer (period 2×spacing, phase drifts with z so layers interlock), segments chained into polylines and clipped exactly to the region. Multi-direction patterns space each set wider (grid ×2, triangles ×3) so density matches `Lines`
- [x] **G2/G3 arc fitting** (opt-in, `arc_fitting` + `arc_tolerance_mm`): the emitter folds runs of toolpath points that lie on a circle into a single G2/G3 (smaller g-code, smoother motion; pairs with arc overhangs). Greedy circle fit per run; a run only qualifies if every chord *hugs* the arc (sagitta ≤ tolerance), so concyclic polygon corners (e.g. a square) stay straight — only genuine curves/rounded-corners convert. Center refit on endpoints+midpoint for accurate I/J; arc-length E. GUI toggle + tol (Quality), CLI `--arc-fitting`. Needs firmware arc support (Klipper `[gcode_arcs]`). Verified: holeplate circles → G2/G3, cube walls stay G1 (only rounded skirt corners arc); unit test (circle yes, square no).
- [ ] 3MF load (zip + XML): multi-object, transforms, embedded settings

### M6 — Advanced / parity
- [x] **Half-height walls** (`half_height_outer_walls` / `--half-outer-walls`; since 2026-06-09 inner walls split too — see below, 2026-06-09 — a feature the mainstream slicers don't have): the outer wall prints as **two half-height passes per layer, each sliced at its own quarter-height plane**, so on sloped surfaces the two contours differ and the visible Z staircase halves (0.2 mm layers → 0.1 mm surface steps) while interior walls/infill keep full layer height. Lower pass (z −h/2) prints first via ascending z-phase ordering (the nozzle never descends into material printed this layer); falls back to the layer outline on degenerate planes; layer 0 stays one fat bead for squish. Per-path `height_scale` drives flow (stadium area at h/2), the volumetric clamp, estimates, and the 3D preview. Mutually exclusive with brick layering (the lower pass would graze brick-lifted beads — GUI grays the other out, engine normalizes). Cost: roughly the outer-wall time again (Benchy 38m34 → 47m52, +1.5% filament — the two half-beads also out-fill the wall's inter-layer cusps slightly, smoothing the face)
- [x] **Brick layering** (opt-in, `brick_layers` + `brick_flow`): odd-indexed perimeters are lifted half a layer height (outer wall = index 0 stays put) so adjacent wall rings interlock like masonry — staggered inter-layer seams resist delamination. The lifted rings get a flow bump (default 1.05) to fuse into the valley; first/last layers are a flat transition/clamp. Per-path `z_offset_mm`/`flow` on `ToolPath`, honored by the emitter (Z + extrusion) and the 3D preview (beads shown lifted + fatter). **Brick-aware motion:** the planner prints the on-plane (low) phase fully before the lifted (high) phase (`order_layers` groups by Z phase), and any travel into/out of a lifted perimeter is forced to **retract + Z-hop** clear (≥ a full bead) so the nozzle never drags through a bead at the other Z — no more per-ring Z-bobbing. CLI `--brick`.
- [x] **Half-height inner walls** (2026-06-09): with half-height walls on, *every* wall — classic loops and arachne beads alike — prints as two half passes from its own quarter-plane contour (arachne runs its field per half). Upper inner walls track the shrinking surface, so nothing stands proud on shallow treads; each phase is contained in its own outer pass (regression test asserts per-phase containment + 0.5 height scale)
- [ ] Tree / organic supports
- [x] **Arachne-class variable-width walls** (`wall_mode = arachne`, the default — demoted to experimental mid-hardening 2026-06-09, re-promoted the same day after the preview-review defect ledger closed: scribbles (skeleton-walk tracing), fragmentation & corner chords (Kuipers §6.2 transition ramps via target-field diffusion + domain dilation), missing arcs (Zhang–Suen thinning for center-bead zones), wandering centerlines (per-vertex snap to the depth ridge, widths from measured thickness); validated against real Benchy layers via `engine --example dbg_arachne`, 93–102% ring coverage; `classic` = old fixed offsets): the **distance-field formulation** of Kuipers et al.'s adaptive beading — region rasterized at ¼ line width, exact Felzenszwalb EDT for depth, ridge cells as the skeleton, nearest-skeleton transform for local thickness, then a beading scheme (`round(2T̂/sp)` capped at wall count) whose bead centerlines are **level sets of the depth field** at `(i+½)·pitch`, extracted by marching squares — beads stretch, squeeze, split and merge with the geometry; odd stretch-zones get a skeleton-traced center bead; thin features (< 1 line width) become single tapered `ExternalPerimeter` beads. Per-vertex widths flow through E, estimates, the flow clamp (max width), and the 3D preview; arc fitting skips variable paths. The outer wall stays a classic exact loop (dimensional accuracy; fuzzy/seam/half-height keep working), saturated zones reproduce classic spacing exactly so the infill interior is unchanged, and gap fill is **binary with arachne** (2026-06-09): the beading scheme's *absorb* regime widens capped rings to swallow sub-bead infill remainders, so arachne mode forces gap fill off entirely; gap fill remains classic mode's companion. Brick + vase force classic. *Scope note: grid-quantized (lw/4) rather than exact skeletal trapezoidation; an exact-arithmetic skeleton (boost::polygon-style segment Voronoi, per Kuipers et al. 2020 §3) remains the upgrade path. The 2026-06-09 hardening pass implemented the paper's **transition ramps** (§6.2; Cura's `wall_transition_length` ≈ 1×lw) as diffusion of the per-bead target field with domain dilation (beads extend into transitions like Cura's anchors), kept the nearest-skeleton thickness partition sharp (one border per junction — blurring it spawned intermediate bead-count slivers), and traces center beads along skeleton cells only (tracing zone areas was the scribble bug). Verified by fixtures: curved-band contiguity (≤4 beads, ≥90mm), disc-strip junction (≤8 beads, full reach), thin-fin taper, beading regimes.* Benchy: 38m34 → 43m19, +2% filament (former voids now printed)
- [x] **Arachne exact graph walk** (2026-06-10, `engine/src/skeletal.rs`): the grid extractor replaced as the primary path by the paper's **skeletal trapezoidation** — segment Voronoi diagram via the `boostvoronoi` crate (eadf's Rust port of boost::polygon::voronoi, the library CuraEngine uses; integer µm input), interior edges + ribs into a half-edge graph (parabolas discretized at 0.8 mm with transition-angle marking points), central-edge marking at the **paper's 45° transitioning angle** (Cura's 10° default shattered thin polygonized rings — measured slopes: legit count-transition climbs 0.10–0.13 vs corner ribs 0.99; `sin 22.5°` splits them with margin), bead counts from the same wall.rs scheme (stretch/absorb/absorb-2/saturated, thresholds unchanged → plan.rs infill gate still in sync), §6.2 transition mids/ends as graph nodes (anchor ribs), extra ribs at the saturation kink, beading propagation up/down with ≈1·lw merge ramp, junctions per upward edge, and the quad/domain walk emitting per-index chains stitched by `join_beads`. Algorithms ported from CuraEngine's `SkeletalTrapezoidation` (clean reimplementation, AGPL output — see decision log); the grid `Field` stays as automatic fallback for degenerate input (`ARACHNE_GRID=1` forces it; ~1 Benchy layer in 268 falls back — a near-tangent hole/notch pinch at the bow). Input hygiene at µm scale: ≤5µm point merge + exactly-collinear vertex removal (collinear touching segments make boost emit the shared endpoint's cell unbounded — no finite vertex to terminate the cell walk). Never panics into the rayon pool: structural validation (unpaired half-edges → fallback) + `catch_unwind`. **Both open defects fixed**: chimney C-ring (layer 218: was 3 fragments + a dot, now one closed 13.8 mm ring) and corner blobs (layer 50: was 12 beads / 8 open width-capped dabs, now 4 closed beads, max width 0.56). Whole-model census: open beads 574 → 131, open length 9.3% → 4.8%; layer 93 coverage 98% (all closed); layer 195's "83%" was a metric artifact (classic reference double-counts a degenerate near-zero-width ring — the walk's single centered 0.44 mm bead is the correct cover, 99.4% vs honest reference). Perf: full Benchy slice **0.55 s vs grid-arachne 2.3 s** (4×; classic 0.31 s). New fixtures: annulus ring closure, L-junction no-pileup (width cap + length budget + vertex-density), strip ring, thin taper, scheme parity.
- [ ] Multi-material / multi-color (sequencing, prime tower)
- [x] **Ironing** (`ironing` + flow/spacing/speed): surfaces with open air above get a final low-flow (15%) fine-spaced (0.15mm) boustrophedon pass at 45°, per island, always ordered after everything else on the layer — melts top ridges flat
- [x] **Fuzzy skin** (`fuzzy_skin` + thickness/point-dist): external perimeters (not layer 0) resampled every ~0.8mm and jittered ±thickness/2 along the local outward normal (works on holes too); deterministic xorshift per layer so slicing is reproducible
- [x] **Spiral vase** (`spiral_vase` / `--vase`): forces 1 wall / no infill / no shells above the solid bottom, then emits each layer's single loop with Z rising continuously along its length (`G1 X Y Z E`) — one seamless helix, no layer-change retractions; falls back to normal emission on layers that aren't a single loop
- [x] **Elephant-foot compensation** (`elephant_foot_mm`) and **XY size compensation** (`xy_compensation_mm`) — first-layer shrink and global grow/shrink applied to the sliced outlines before planning
- [x] **Cooling & flow control**: part-fan duty (`fan_speed`), fan off for first N layers, bridge fan override, bridge flow ratio, global extrusion multiplier (filament tier), Klipper `SET_PRESSURE_ADVANCE` emission (filament tier, opt-in)
- [ ] Scarf seams
- [ ] Network printing + monitoring

---

## Cross-cutting workstreams

**Testing & correctness**
- [ ] Golden-file snapshots for slices and (later) g-code; fail on diff
- [ ] Property tests: sliced area ≈ cross-section; loops closed; winding consistent
- [ ] Determinism: no `HashMap` iteration order in engine output (use sorted / `BTreeMap` / `IndexMap`)
- [ ] Corpus of nasty meshes (non-manifold, self-intersecting, open) the slicer must survive

**Performance** (defer until correct)
- [x] Bucket triangles by z-span (avoid O(layers × triangles)) — banded buckets, ~28× faster slicing on Benchy
- [x] `rayon` across layers — slicing, planning passes 1+2, support overhang precompute, travel combing (per-layer entry states derived sequentially first), min-layer-time
- [x] Arc-overhang grid classification rasterized by scanline (was point-in-polygon per cell)
- [x] G-code builder writes with `fmt::Write` into one pre-sized buffer (no per-line temporaries)
- [ ] Avoid needless allocation in the hot path (further: stitch hash maps, clipper round-trips)

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
- **Per-feature speeds derive from the machine's print speed** when a profile
  doesn't pin them (external 50%, solid 80%, support 90%, gap fill 40% ≤ 40mm/s).
  Absolute defaults would silently slow fast printers (a Voron at 150mm/s print
  was getting a 25mm/s outer wall) — discovered via the Benchy time-estimate
  regression, caught by histogramming extrusion length per feed rate.
- **Monotonic fill is per-island.** A strict sweep across a whole *region* makes
  travel ping-pong between disjoint islands on every scanline; sheen only needs
  monotonic order per contiguous surface. `ToolPath::group` marks indivisible
  blocks for the travel orderer; distinct islands stay independently orderable.
- **Stadium flow + spacing, locked in (no model knob).** The bead is physically
  a stadium; flow and placement both derive from it in one place
  (`config::bead_area_mm2` / `bead_spacing_mm`), keeping solid surfaces
  area-exact (`A / spacing / h = 1`) and density semantics intact
  (`spacing / density`). **Migration note:** vs. the old rectangle model,
  single beads extrude ~9.5% less (Benchy −5.7% filament overall) — anyone who
  had tuned `extrusion_multiplier` below 1.0 to compensate should re-tune
  toward 1.0. Per the legibility rules there is deliberately no
  rectangle/stadium toggle.
- **Optimizer legibility rules** (agreed 2026-06-09, applies to all "smart"
  balancing features): (1) *physics is not a preference* — single-correct
  computations (bead cross-section model, flow math) are locked in, with the
  global calibration escape (`extrusion_multiplier`) preserved, never exposed
  as a model-choice knob; (2) *limits act loudly* — safety rails like the
  volumetric clamp default on, the user controls the limit value, and every
  intervention is reported (g-code header, CLI, GUI status), never silent;
  (3) *derivations are visible* — derived defaults (per-feature speeds, later
  line-width-from-nozzle) display as "auto" with the live computed value and
  pin to manual when touched; (4) *nothing rewrites saved profiles behind the
  user's back* — diff-based saves guarantee un-pinned auto values are never
  serialized. No global simple/expert mode switch; progressive disclosure via
  the tier-colored collapsible sections instead.
- **Gap fill = chained-offset comparison + PCA strokes.** Per wall w, gap =
  (material at depth w·lw) − (dilated wall-w centerlines); plus open-dropped
  infill slivers. Filter hair ribbons (open by 0.15·lw, stroke width ≥ 0.3·lw),
  then fill each island with single strokes along its principal axis, width
  matched to 2·area/perimeter. Simple, robust, no medial axis needed (yet).

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
