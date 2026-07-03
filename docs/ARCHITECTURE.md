# Architecture

How the slicer is put together and why. Progress and milestones live in
[../PLAN.md](../PLAN.md).

## Shape of the system

A **headless engine library + thin front-ends**, the same separation CuraEngine
has from Cura. The `engine` crate knows nothing about UIs or files beyond
producing geometry/g-code; the `cli` and `gui` are consumers. This keeps
the core testable and is what structurally keeps us independent of a monolith.

```
              ┌─────────┐
   STL/3MF ──▶│  mesh   │ indexed triangles, STL/3MF I/O
              └────┬────┘
                   ▼
              ┌─────────┐     ┌────────┐
              │ engine  │◀────│ geo2d  │ integer polygons, Clipper2
              └────┬────┘     └────────┘
                   │  layers → walls → infill → surfaces → supports → toolpaths
                   ▼
              ┌─────────┐     ┌────────┐
              │  gcode  │◀────│ config │ printer/filament/process profiles
              └────┬────┘     └────────┘
                   ▼
            .gcode / SVG  ◀── cli (`fable-slicer-cli`) · gui (`fable-slicer`, egui/wgpu)
```

## The pipeline

1. **Load & repair** — STL/3MF → indexed mesh; tolerate imperfect input. A 3MF
   keeps its per-part identity (leaf meshes, names, and extruder hints from
   Bambu/Orca's `Metadata/model_settings.config`).
2. **Slice** — intersect each z-plane with the mesh → closed layer polygons.
   Multi-part models slice per part on ONE shared layer grid
   (`engine::generate_parts`): each part plans its own walls/skins/infill
   (interfaces between parts get full shells), physical tests (overhang,
   bridge, support) run against the per-layer union of all parts, and layers
   merge with paths grouped by tool for toolchange-minimal ordering.
3. **2D ops** — boolean + offset on polygons (Clipper2).
4. **Walls** — concentric inward offsets.
5. **Infill** — clip a pattern to the wall interior; solid vs. sparse regions.
6. **Surfaces** — top/bottom detection, bridging, ironing.
7. **Supports** — overhang detection; grid supports.
8. **Toolpaths** — order regions (travel minimization), seams, combing.
9. **Extrusion + motion** — geometry → E values, speeds, cooling.
10. **G-code** — emit moves, retraction, fan, arc fitting; simulate for time.
11. **Preview** — feature-colored path rendering (GUI).

All eleven steps are implemented. Supports are grid-based (no tree supports),
and robust mesh repair for messy input is still the main gap (see PLAN.md).
Steps 2, 4–8 run layer-parallel on rayon.

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

Cut segments are **directed from the triangle winding** (material on the
left), so stitched outers come out CCW and holes CW straight from the
geometry, and each layer is then normalized under the **positive fill rule**
— material where the winding count is > 0. This is what makes geometrically
self-intersecting meshes slice correctly: a chamfer surface punching through
a wall (topologically manifold, so no repair pass flags it) yields crossing
contours whose nesting parity is meaningless, but whose winding still knows
where the material is. A mesh with enough flipped facets that the directed
walk can't close its loops falls back per layer to the tolerant undirected
stitcher (orientation by nesting parity), then normalizes the same way.
This deliberately avoids a half-edge structure — integer snapping gives
exact connectivity for clean meshes.

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
