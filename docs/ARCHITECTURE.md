# Architecture

How the slicer is put together and why. Progress and milestones live in
[../PLAN.md](../PLAN.md).

## Shape of the system

A **headless engine library + thin front-ends**, the same separation CuraEngine
has from Cura. The `engine` crate knows nothing about UIs or files beyond
producing geometry/g-code; the `cli` (and later `gui`) are consumers. This keeps
the core testable and is what structurally keeps us independent of a monolith.

```
              ┌─────────┐
   STL/3MF ──▶│  mesh   │ indexed triangles, STL/3MF I/O
              └────┬────┘
                   ▼
              ┌─────────┐     ┌────────┐
              │ engine  │◀────│ geo2d  │ integer polygons, (M1) Clipper2
              └────┬────┘     └────────┘
                   │  layers → walls → infill → surfaces → supports → toolpaths
                   ▼
              ┌─────────┐     ┌────────┐
              │  gcode  │◀────│ config │ printer/filament/process profiles
              └────┬────┘     └────────┘
                   ▼
            .gcode / SVG  ◀── cli (binary: `slicer`),  later: gui (egui/wgpu)
```

## The pipeline (target end state)

1. **Load & repair** — STL/3MF → indexed mesh; tolerate imperfect input.
2. **Slice** — intersect each z-plane with the mesh → closed layer polygons.
3. **2D ops** — boolean + offset on polygons (Clipper2).
4. **Walls** — concentric inward offsets; later Arachne variable width.
5. **Infill** — clip a pattern to the wall interior; solid vs. sparse regions.
6. **Surfaces** — top/bottom detection, bridging, ironing.
7. **Supports** — overhang detection; grid then tree supports.
8. **Toolpaths** — order regions (travel minimization), seams, combing.
9. **Extrusion + motion** — geometry → E values, speeds, cooling.
10. **G-code** — emit moves, retraction, fan, arc fitting; simulate for time.
11. **Preview** — feature-colored path rendering (GUI).

Implemented today: all eleven steps in v1 form (Arachne walls, tree supports,
and mesh repair remain; see PLAN.md). Steps 2, 4–8 run layer-parallel on rayon.

## Coordinate system

The engine works in **integer "Clipper space"**: `i64` nanometers, `1 mm =
1_000_000` units (`geo2d::UNITS_PER_MM`). Integers make polygon booleans/offsets
exact and let shared vertices compare bit-for-bit, so contours stitch without
epsilon matching. Floating-point millimeters appear only at the boundary: mesh
coordinates on the way in, g-code/SVG on the way out.

Winding convention: **outer loops CCW, holes CW** (positive shoelace area =
outer). It's enforced after stitching from nesting parity, so facet orientation
in the source mesh doesn't matter.

## Slicing in detail (current code)

`engine::slice_mesh` samples each layer at its vertical **midpoint**
(`z = zmin + h*(i + 0.5)`) to avoid landing on flat top/bottom facets; the plane
is walked off coincident vertices (1 µm bumps against a sorted unique-z table)
so no triangle vertex sits exactly on it. Triangles are **bucketed by the band
of layers their z-span crosses** — each layer visits only candidate triangles,
not the whole mesh — and the layers are sliced **in parallel** (rayon). For each
straddling triangle, the lone vertex (alone on its side of the plane) defines
the two crossing edges; we interpolate the two intersection points and snap them
to the integer grid. All segments are stitched **undirected**: on a manifold
mesh each cut point has degree two, so the segments form simple cycles we can
walk directly. Direction is irrelevant because winding is fixed afterward.

This deliberately avoids a half-edge structure for now — integer snapping gives
exact connectivity for clean meshes. Topology-aware stitching is a later
robustness upgrade for messy inputs (see PLAN.md → M0 robustness pass).

## Why these dependencies

- **Clipper2** (Boost license) for 2D offset/boolean — the robustness-critical
  piece we will not reinvent.
- **rayon** for per-layer parallelism — slicing is embarrassingly parallel and
  this is where a from-scratch Rust engine can beat the C++ incumbents.
- **clap / anyhow** for the CLI ergonomics.
- **zip + quick-xml** for 3MF import — the container is a zip of XML parts.
- **egui + wgpu** (GUI/preview), **serde** (profiles).

## Determinism

Engine output must be reproducible (golden tests, repeatable slices). That means
no reliance on `HashMap` iteration order in anything that affects output ordering
— use sorted iteration, `BTreeMap`, or `IndexMap`. The current stitcher seeds
walks in segment input order; the set of loops produced is order-independent.
