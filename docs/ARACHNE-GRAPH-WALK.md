# Handoff: Arachne graph walk (exact skeletal trapezoidation)

> **STATUS: IMPLEMENTED 2026-06-10** — `engine/src/skeletal.rs`. Both defect
> classes fixed; see the completion report at the bottom of this file. The
> grid extractor survives as the automatic fallback for degenerate input
> (`ARACHNE_GRID=1` forces it for A/B).

Resume artifact, written 2026-06-10 at the end of the grid-arachne hardening
session. Goal: replace the grid extractor's *piecewise extraction + reassembly*
with the paper's graph walk, which produces rings as rings and junctions as
nodes — eliminating the two remaining defect classes.

## Why (the engineering verdict)

The grid formulation (current `engine/src/wall.rs`) now works on ~95% of
geometry (user-confirmed on Benchy), but it extracts beads as **pieces** —
marching-squares level sets where the band classifies ≥2 beads, thinned-
skeleton traces where it classifies 1 — and sews them with joins/ramps/
dilations. Topology breaks the sewing:

- **Open defect 1 — C-ring:** chimney annulus, Benchy **layer ~218**: the
  inner wall should be closed ring(s); classification flips around the
  circumference and the pieces don't reassemble (~90° gap).
- **Open defect 2 — corner blobs:** hull corner, Benchy **layer ~50**: beads
  from different regimes pile up at a junction instead of one bead flowing
  through.

A skeletal-trapezoidation **graph walk** fixes both *by construction*.

## What to build (and only this — the rest survives)

Reference: Kuipers, Doubrovski, Wu, Wang 2020, *A framework for adaptive
width control of dense contour-parallel toolpaths* (§3 skeletal
trapezoidation, §5 toolpath extraction, §6.2 transitions). Reference code:
CuraEngine `src/SkeletalTrapezoidation.cpp`, `SkeletalTrapezoidationGraph`,
`BeadingStrategy/`, `WallToolPaths` (read for algorithms, port no code — AGPL
is fine here anyway, see PLAN decision log). *[2026-06-12 audit note: the
implementation ended up a structural port of the Cura code, not a clean
reimplementation — now attributed to UltiMaker in `skeletal.rs`'s file
header; see the completion report below and the PLAN decision log.]*

1. **Segment Voronoi diagram** of the layer polygon (sites = boundary segments
   + vertices, diagram restricted to the interior = medial axis).
   - FIRST: try the `boostvoronoi` crate (eadf's Rust port of
     boost::polygon::voronoi — the exact library CuraEngine uses). Check
     `ls ~/.cargo/registry/cache/*/ | grep -i voronoi` and the sparse index;
     crates.io HEAD 403s but fetches work. If usable, step 1 is nearly free.
   - Fallback: implement Fortune-with-segments (weeks; avoid if crate works).
   - Inputs are our integer nm coordinates (`geo2d::Point`) — boostvoronoi
     wants i32/i64 integer input, which matches our representation exactly.
2. **Skeletal trapezoidation graph**: from Voronoi edges interior to the
   polygon, build graph nodes (Voronoi vertices, with radius = distance to
   boundary) and edges; discard edges whose cells belong to reflex-vertex
   pairs per the paper; quantize/merge near-duplicate nodes (integer coords
   help).
3. **Beading along the graph**: reuse `wall.rs` scheme verbatim — `Scheme`
   {stretch / absorb / absorb-2 / saturated} with the same thresholds
   (sliver = 1.444·sp, +1 center bead window + sp/0.9, pitch clamp 1.7·sp,
   width clamp via `width_of` = pitch + (lw − sp), 1.75·lw cap). Bead count
   from node radius R: t = 2R.
4. **Junction placement + transitions**: per graph edge, compute bead
   "junctions" (position along rib, width) at each bead index; insert
   transition anchors where the bead count changes (§6.2; transition length
   ≈ 1·lw — same constant the grid version diffuses by).
5. **Toolpath extraction**: walk the graph per bead index, connecting
   junctions into polylines/rings (the graph's connectivity IS the bead
   connectivity — no join pass needed). Output `wall::Bead { points, widths,
   closed }` — the existing interface.

### Integration point (one function)

`engine/src/wall.rs :: variable_walls(outer, inner, lw, sp, max_inner) ->
VariableWalls`. Swap its internals; keep the grid `Field` path behind a
fallback (e.g. when Voronoi fails on degenerate input) or delete once stable.
Everything downstream is already done and tested:

- per-vertex widths through emitter (per-segment E via stadium area), flow
  clamp (max width), estimates, GUI preview (tapered beads), SVG
- saturation-consistent infill gate in `plan.rs` (the `arachne &&` branch
  computing `inset` via morphological open at r = lw + cap·sp + 1.278·sp —
  keep thresholds in sync with the scheme!)
- thin-feature beads (< 1·lw) as ExternalPerimeter; outer wall stays classic
  Clipper loops (+ half-height variants); brick/vase force classic;
  `wall_mode` setting (currently default Arachne), GUI/CLI/profiles wired

## Acceptance (all already runnable)

1. `cargo test --workspace` green (49+ tests; wall fixtures: curved-band
   contiguity ≤4 beads ≥90mm, disc-strip junction ≤10 beads + reach, frame
   narrow band, thin fin taper, scheme regimes, wedge per-phase coverage).
2. `ARACHNE_DBG=1 cargo run --release -p engine --example dbg_arachne` —
   ring coverage vs classic on real Benchy layers 93/195: target ≥97% both
   (grid version: 99.5% / 83%).
3. **New fixtures to add**: annulus at chimney dimensions asserting all beads
   closed (defect 1); L-junction asserting no bead self-overlap / pile-up
   (defect 2 — e.g. max points-per-mm density along each bead).
4. User preview check on Benchy layers 50 / 177 / 195 / 207 / 218 (the
   session's crime scenes), arachne + half-height on.

## Context you'd otherwise rediscover

- Cell = lw/4 grid; sp = `config::bead_spacing_mm` (stadium spacing — flow
  math assumes beads *placed* at sp; the scheme's width_of inverts it).
- `join_beads` (gap ≤ 0.8·lw) and the target-field diffusion (`blur_finite`,
  `blur_passes ≈ lw/cell`) exist to patch what the graph walk obsoletes —
  delete them with the grid path.
- Zhang–Suen `thin_mask` stays useful for the thin-feature (< lw) ridge even
  in the graph version, unless Voronoi handles those too (it does — prefer it).
- History of failure modes (don't regress): zone-area tracing → scribbles;
  blurred t̂ partition → bead-count sliver fans; plateau masking → fragmented
  beads; ±0.25mm snap → insufficient. All fixed; commits `149e74f`…`1e140d9`
  tell the story.
- User priorities: GUI is the primary binary; optimizer legibility rules in
  PLAN decision log; nothing has been test-printed yet.

---

## Completion report (2026-06-10)

Implemented in `engine/src/skeletal.rs` (~1700 lines), a structural port of
CuraEngine's `SkeletalTrapezoidation` adapted to the wall.rs scheme (the
2026-06-12 provenance audit found it closer to a port than the "port no code"
plan above — UltiMaker attribution added to the file header). Step-by-step
outcome against the plan above:

1. **Voronoi**: `boostvoronoi 0.12` worked as hoped — integer µm input
   (nm/1000), `Builder::<i64>::with_segments`. Two input-hygiene passes were
   required at µm scale: merge points closer than 5µm (micro-segments starve
   the cell walk) and strip *exactly-collinear* vertices (boost emits the
   shared endpoint's cell as two infinite secondary edges — no finite vertex
   exists for the cell-range walk to terminate on; Cura never sees this
   because its input mending strips them first).
2. **Graph**: index-arena half-edge port of constructFromPolygons /
   transferEdge / discretize (parabolas + point-point bisectors, 0.8mm step,
   marking points) / makeRib / separatePointyQuadEndNodes /
   collapseSmallEdges. Structural validation (unpaired half-edges) plus
   `catch_unwind` guarantee the path can only ever *fall back*, never take
   the rayon pool down.
3. **Beading**: wall.rs scheme verbatim behind Cura's strategy interface
   (optimal count / transition thickness / compute / transition length 1·lw /
   anchor / nonlinear thicknesses = the saturation kink, pinned by an extra
   rib).
4. **Transitions**: full §6.2 port (mids → filtering/dissolving → ends →
   `insert_node` anchor ribs). One parameter departed from Cura: the central
   marking angle is the **paper's 45°** (δmax = 135°), not Cura's 10°
   default. Measured on the Benchy chimney (layer 218): legitimate
   count-transition climbs have local slopes 0.10–0.13 (polygonization
   concentrates the medial-axis climb at vertices) vs. true corner ribs at
   0.99; sin(5°) = 0.087 cuts *into* the first population and shattered the
   ring; sin(22.5°) = 0.38 splits the two with margin on both sides.
5. **Extraction**: junctions per upward edge from the top node's beading;
   beading propagation up/down with the 1·lw merge ramp; quad/domain walk
   with odd-bead dedup and 3-way handling; local-maxima dots;
   `join_beads(0.8·lw)` as the stitcher. Thin features (< 1·lw) come from
   central chains with R < lw/2 on the outer region — Voronoi handles them,
   as predicted; Zhang–Suen stays grid-only.

**Acceptance**: 76 workspace tests green (new fixtures: annulus all-closed,
L-junction no-pileup, strip ring, thin taper, scheme parity). Defect 1
(layer 218 C-ring): one closed 13.8mm ring (was 3 fragments + a dot).
Defect 2 (layer 50 corner blobs): 4 closed beads, max width 0.56 (grid: 12
beads, 8 open width-capped dabs). Whole-model census: open beads 574 → 131,
open length 9.3% → 4.8%. Layer 93 coverage 98.0%, all closed. Layer 195's
"83%" was a *metric artifact*: its chimney band leaves a 0.4mm channel whose
classic reference is two loops 0.036mm apart (double-counted, massively
overextruded); the walk's single centered 0.44mm bead is the correct cover —
99.4% against the honest reference. **Perf: full Benchy 0.55s vs 2.3s
grid-arachne (4×), classic 0.31s.** One Benchy layer in 268 falls back to
grid (near-tangent hole/notch pinch at the bow after the outer-wall inset —
the intended fallback class).

Not done / future: delete the grid path + `blur_finite` once the fallback
proves unnecessary in the field; gate the thin-feature voronoi pass on a
cheap thinness test (it runs on every layer's full outline); user preview
check on real prints.
