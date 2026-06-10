# Handoff: Arachne graph walk (exact skeletal trapezoidation)

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
is fine here anyway, see PLAN decision log).

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
