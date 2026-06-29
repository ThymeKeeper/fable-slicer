//! Clipper2-backed polygon offsetting and boolean operations.
//!
//! We use the `One` scaler (multiplier 1.0) so our integer nanometer coordinates
//! pass straight through Clipper without a second round of scaling: `from_scaled`
//! stores them verbatim and `Into<Vec<Vec<(f64,f64)>>>` reads them back exactly
//! (nm values are well under 2^53, so the f64 round-trip is lossless).

use clipper2::{
    difference as cl_difference, intersect as cl_intersect, union as cl_union, EndType, FillRule,
    JoinType, One, Paths, Point as CPoint,
};

use crate::{Contour, Point, Polygons, UNITS_PER_MM};

fn to_paths(polys: &Polygons) -> Option<Paths<One>> {
    let rings: Vec<Vec<CPoint<One>>> = polys
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
    if rings.is_empty() {
        None
    } else {
        Some(rings.into())
    }
}

fn from_paths(paths: Paths<One>) -> Polygons {
    let rings: Vec<Vec<(f64, f64)>> = paths.into();
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

/// Offset (inflate / deflate) a polygon set by `delta_mm`. Positive grows
/// outward, negative shrinks inward. Holes are handled via the input winding
/// (outer CCW, hole CW), so a negative delta correctly erodes the solid region.
pub fn offset(polys: &Polygons, delta_mm: f64) -> Polygons {
    let Some(paths) = to_paths(polys) else {
        return Polygons::new();
    };
    let delta = delta_mm * UNITS_PER_MM;
    // Round joins avoid spikes on corners; simplify drops the dense near-duplicate
    // points inflate generates (tolerance ~1 µm).
    let result = paths
        .inflate(delta, JoinType::Round, EndType::Polygon, 2.0)
        .simplify(0.001 * UNITS_PER_MM, false);
    from_paths(result)
}

/// Stroke an OPEN polyline into its bead footprint: the area within
/// `half_width_mm` of the path, rounded caps and joins. Mirrors how a closed
/// loop's bead is stamped (`offset(+r) − offset(−r)`), but for an open arc — e.g.
/// a trimmed perimeter, which has no interior to subtract.
pub fn stroke_open(points: &[Point], half_width_mm: f64) -> Polygons {
    if points.len() < 2 || half_width_mm <= 0.0 {
        return Polygons::new();
    }
    let path: Vec<CPoint<One>> =
        points.iter().map(|p| CPoint::<One>::from_scaled(p.x, p.y)).collect();
    let paths: Paths<One> = vec![path].into();
    let delta = half_width_mm * UNITS_PER_MM;
    // EndType::Round strokes an OPEN path (vs EndType::Polygon for closed loops).
    let result = paths
        .inflate(delta, JoinType::Round, EndType::Round, 2.0)
        .simplify(0.001 * UNITS_PER_MM, false);
    from_paths(result)
}

/// Reduce vertex count, collapsing detail finer than `epsilon_mm`.
pub fn simplify(polys: &Polygons, epsilon_mm: f64) -> Polygons {
    let Some(paths) = to_paths(polys) else {
        return Polygons::new();
    };
    from_paths(paths.simplify(epsilon_mm * UNITS_PER_MM, false))
}

/// Boolean union (`a ∪ b`).
pub fn union(a: &Polygons, b: &Polygons) -> Polygons {
    match (to_paths(a), to_paths(b)) {
        (Some(p), Some(q)) => cl_union(p, q, FillRule::NonZero).map(from_paths).unwrap_or_default(),
        (Some(p), None) => from_paths(p),
        (None, Some(q)) => from_paths(q),
        (None, None) => Polygons::new(),
    }
}

/// Boolean difference (`a − b`).
pub fn difference(a: &Polygons, b: &Polygons) -> Polygons {
    match (to_paths(a), to_paths(b)) {
        (Some(p), Some(q)) => cl_difference(p, q, FillRule::NonZero)
            .map(from_paths)
            .unwrap_or_default(),
        (Some(p), None) => from_paths(p), // a − ∅ = a
        (None, _) => Polygons::new(),     // ∅ − b = ∅
    }
}

/// Boolean intersection (`a ∩ b`).
pub fn intersection(a: &Polygons, b: &Polygons) -> Polygons {
    match (to_paths(a), to_paths(b)) {
        (Some(p), Some(q)) => cl_intersect(p, q, FillRule::NonZero)
            .map(from_paths)
            .unwrap_or_default(),
        _ => Polygons::new(), // intersection with ∅ is ∅
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square_at(x0: f64, y0: f64, side: f64) -> Polygons {
        let mut p = Polygons::new();
        p.push(Contour::new(vec![
            Point::from_mm(x0, y0),
            Point::from_mm(x0 + side, y0),
            Point::from_mm(x0 + side, y0 + side),
            Point::from_mm(x0, y0 + side),
        ]));
        p
    }

    #[test]
    fn negative_delta_shrinks() {
        let inner = offset(&square_at(0.0, 0.0, 20.0), -1.0);
        let area: f64 = inner.contours.iter().map(|c| c.area_mm2()).sum();
        assert!((area - 18.0 * 18.0).abs() < 2.0, "got {area}");
    }

    #[test]
    fn positive_delta_grows() {
        let outer = offset(&square_at(0.0, 0.0, 20.0), 1.0);
        let area: f64 = outer.contours.iter().map(|c| c.area_mm2()).sum();
        assert!(area > 20.0 * 20.0 && area < 22.5 * 22.5, "got {area}");
    }

    #[test]
    fn difference_makes_a_hole() {
        // 20mm square minus a centered 10mm square => net area 300mm².
        let d = difference(&square_at(0.0, 0.0, 20.0), &square_at(5.0, 5.0, 10.0));
        assert!((d.net_area_mm2() - 300.0).abs() < 2.0, "got {}", d.net_area_mm2());
    }

    #[test]
    fn intersection_of_overlap() {
        // Two 20mm squares offset by 10mm overlap in a 10x20 region => 200mm².
        let i = intersection(&square_at(0.0, 0.0, 20.0), &square_at(10.0, 0.0, 20.0));
        let area: f64 = i.contours.iter().map(|c| c.area_mm2()).sum();
        assert!((area - 200.0).abs() < 2.0, "got {area}");
    }
}
