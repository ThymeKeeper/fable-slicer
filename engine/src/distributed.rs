//! Distributed-bead walls — the "piping" generator.
//!
//! Concentric rings laid by iterative inset, with the bead width re-derived
//! from the *remaining* core on every pass, so the slack is shared evenly
//! across all rings: no gap-fill bead, no marching-squares wobble. Each ring is
//! an exact polygon offset, so the curves are clean and the topology (rings
//! splitting and merging as the core pinches off) falls out of the offset for
//! free.
//!
//! This is the offset-native answer to Arachne: the same adaptive bead count
//! and even bead width, but reached by filling an oddly shaped cake from the
//! rim inward rather than by an exact skeletal trapezoidation.

use crate::wall::Bead;
use geo2d::{offset, Polygons};

/// Largest inward inset (mm) that still leaves material — the region's local
/// maximum half-thickness — found by bisection on the offset area. `hi` must be
/// an inset that already empties the region (an upper bound on the inradius).
fn max_inradius(region: &Polygons, mut hi: f64) -> f64 {
    if region.net_area_mm2() <= 0.0 {
        return 0.0;
    }
    let mut lo = 0.0;
    for _ in 0..14 {
        let mid = 0.5 * (lo + hi);
        if offset(region, -mid).net_area_mm2() > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// Inner adaptive rings for `inner` (the region already inset past the outer
/// wall), emitted outermost-first. `max_inner` caps the ring count; the deeper
/// core beyond it is left to infill.
///
/// Each pass measures the half-thickness `t` of what's left, divides the full
/// thickness `2t` into a whole number of even beads, lays one ring at half a
/// bead's depth, then peels a full bead and repeats. Because the pitch is
/// re-derived from the shrinking core, the final ring lands exactly on the
/// spine — there is never a leftover slice to dump into a fat gap-fill bead.
pub(crate) fn distributed_rings(inner: &Polygons, lw: f64, sp: f64, max_inner: usize) -> Vec<Bead> {
    let mut beads = Vec::new();
    if max_inner == 0 {
        return beads;
    }
    let bb = match inner.bounds() {
        Some(b) => b,
        None => return beads,
    };
    // The inradius can't exceed half the shorter bounding span.
    let span = (bb.max.x_mm() - bb.min.x_mm()).min(bb.max.y_mm() - bb.min.y_mm());
    // Centerline pitch → extruded width: a bead spaced `pitch` apart must be
    // `pitch + (lw - sp)` wide to keep the same overlap a nominal bead has.
    let width_of = |pitch: f64| (pitch + (lw - sp)).clamp(lw * 0.5, lw * 1.75);

    let mut hi = 0.5 * span + sp;
    let mut remaining = inner.clone();
    for _ in 0..max_inner {
        let t = max_inradius(&remaining, hi);
        if t < sp * 0.35 {
            break; // nothing printable left in the core
        }
        let n = ((2.0 * t / sp).round() as usize).max(1);
        let pitch = (2.0 * t / n as f64).clamp(sp * 0.5, sp * 1.7);
        let width = width_of(pitch);

        let center = offset(&remaining, -0.5 * pitch);
        for c in &center.contours {
            if c.points.len() >= 3 {
                beads.push(Bead {
                    widths: vec![width; c.points.len()],
                    points: c.points.clone(),
                    closed: true,
                });
            }
        }

        remaining = offset(&remaining, -pitch);
        if remaining.net_area_mm2() <= 0.0 {
            break;
        }
        hi = t; // the inradius only shrinks as we peel inward
    }
    beads
}
