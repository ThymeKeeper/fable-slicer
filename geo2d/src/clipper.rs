//! Clipper2-backed polygon offsetting.
//!
//! We use the `One` scaler (multiplier 1.0) so our integer nanometer coordinates
//! pass straight through Clipper without a second round of scaling: `from_scaled`
//! stores them verbatim and `Into<Vec<Vec<(f64,f64)>>>` reads them back exactly
//! (nm values are well under 2^53, so the f64 round-trip is lossless).

use clipper2::{EndType, JoinType, One, Paths, Point as CPoint};

use crate::{Contour, Point, Polygons, UNITS_PER_MM};

/// Offset (inflate / deflate) a polygon set by `delta_mm`. Positive grows
/// outward, negative shrinks inward. Holes are handled via the input winding
/// (outer CCW, hole CW) — which the slicer guarantees — so a negative delta
/// correctly erodes the solid region.
pub fn offset(polys: &Polygons, delta_mm: f64) -> Polygons {
    let subject: Vec<Vec<CPoint<One>>> = polys
        .contours
        .iter()
        .filter(|c| c.points.len() >= 3)
        .map(|c| {
            c.points
                .iter()
                .map(|p| CPoint::<One>::from_scaled(p.x, p.y))
                .collect()
        })
        .collect();
    if subject.is_empty() {
        return Polygons::new();
    }

    let paths: Paths<One> = subject.into();
    let delta = delta_mm * UNITS_PER_MM;

    // Round joins avoid spikes on corners; simplify drops the dense near-duplicate
    // points inflate generates (tolerance ~1 µm).
    let result = paths
        .inflate(delta, JoinType::Round, EndType::Polygon, 2.0)
        .simplify(0.001 * UNITS_PER_MM, false);

    let rings: Vec<Vec<(f64, f64)>> = result.into();
    let contours = rings
        .into_iter()
        .map(|ring| {
            Contour::new(
                ring.into_iter()
                    .map(|(x, y)| Point::new(x.round() as i64, y.round() as i64))
                    .collect(),
            )
        })
        .filter(|c| c.points.len() >= 3)
        .collect();
    Polygons { contours }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(side_mm: f64) -> Polygons {
        let mut p = Polygons::new();
        p.push(Contour::new(vec![
            Point::from_mm(0.0, 0.0),
            Point::from_mm(side_mm, 0.0),
            Point::from_mm(side_mm, side_mm),
            Point::from_mm(0.0, side_mm),
        ]));
        p
    }

    #[test]
    fn negative_delta_shrinks() {
        let s = square(20.0);
        let inner = offset(&s, -1.0); // 1mm inward => 18mm square
        let area: f64 = inner.contours.iter().map(|c| c.area_mm2()).sum();
        assert!((area - 18.0 * 18.0).abs() < 2.0, "got {area}");
    }

    #[test]
    fn positive_delta_grows() {
        let s = square(20.0);
        let outer = offset(&s, 1.0); // ~22mm square (rounded corners)
        let area: f64 = outer.contours.iter().map(|c| c.area_mm2()).sum();
        assert!(area > 20.0 * 20.0 && area < 22.5 * 22.5, "got {area}");
    }
}
