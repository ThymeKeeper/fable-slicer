//! The slicer core.
//!
//! Current scope (M0): mesh -> per-layer closed polygons. Walls, infill, surface
//! detection, supports, toolpath ordering, and g-code emission attach to this
//! pipeline in later milestones (see PLAN.md).
//!
//! ## Slicing algorithm
//!
//! For each horizontal plane `z`, every triangle that straddles the plane
//! contributes one line segment (its intersection with the plane). Those
//! segments are then stitched end-to-end into closed loops. We stitch using the
//! integer endpoints directly: because intersection points are snapped to the
//! shared nanometer grid, a point computed from two adjacent triangles is
//! *bit-identical*, so loop connectivity is exact — no epsilon matching.
//!
//! Winding is fixed after stitching from signed area + nesting parity (outer
//! loops CCW, holes CW), which is robust regardless of the mesh's facet
//! orientation.

mod slice;

pub use slice::{slice_mesh, Layer, SliceParams};
