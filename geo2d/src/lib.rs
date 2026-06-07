//! 2D integer geometry primitives — "Clipper space".
//!
//! All coordinates are integers measured in **nanometers** (`1 mm = 1_000_000`
//! units). Working in scaled integers is what makes polygon boolean/offset
//! operations robust: there is no floating-point drift, so shared vertices
//! compare exactly and contours stitch without epsilon fudging. Conversion
//! to/from millimeters happens only at the I/O boundary (mesh load, g-code/SVG
//! emit).
//!
//! This crate is deliberately dependency-free and pure. The Clipper2 wrapper
//! (offsetting, booleans) lands here at M1; for now it provides the point /
//! contour / polygon types and the handful of predicates slicing needs.

mod clipper;
pub use clipper::offset;

/// Integer units per millimeter (nanometer resolution).
pub const UNITS_PER_MM: f64 = 1_000_000.0;

/// A single coordinate component.
pub type Coord = i64;

/// Convert millimeters to integer units.
#[inline]
pub fn to_units(mm: f64) -> Coord {
    (mm * UNITS_PER_MM).round() as Coord
}

/// Convert integer units to millimeters.
#[inline]
pub fn to_mm(units: Coord) -> f64 {
    units as f64 / UNITS_PER_MM
}

/// A point in Clipper space.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Point {
    pub x: Coord,
    pub y: Coord,
}

impl Point {
    #[inline]
    pub const fn new(x: Coord, y: Coord) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn from_mm(x: f64, y: f64) -> Self {
        Self { x: to_units(x), y: to_units(y) }
    }

    #[inline]
    pub fn x_mm(self) -> f64 {
        to_mm(self.x)
    }

    #[inline]
    pub fn y_mm(self) -> f64 {
        to_mm(self.y)
    }
}

/// A single closed loop of points. The closing edge (last -> first) is implicit;
/// the first point is not repeated at the end.
#[derive(Clone, Debug, Default)]
pub struct Contour {
    pub points: Vec<Point>,
}

impl Contour {
    pub fn new(points: Vec<Point>) -> Self {
        Self { points }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Signed area in mm² via the shoelace formula. Positive => counter-clockwise.
    /// Computed in f64 (mm) to avoid i64 overflow when summing `x*y` products.
    pub fn signed_area_mm2(&self) -> f64 {
        let n = self.points.len();
        if n < 3 {
            return 0.0;
        }
        let mut acc = 0.0;
        for i in 0..n {
            let a = self.points[i];
            let b = self.points[(i + 1) % n];
            acc += a.x_mm() * b.y_mm() - b.x_mm() * a.y_mm();
        }
        acc * 0.5
    }

    /// Unsigned area in mm².
    pub fn area_mm2(&self) -> f64 {
        self.signed_area_mm2().abs()
    }

    pub fn is_ccw(&self) -> bool {
        self.signed_area_mm2() > 0.0
    }

    pub fn make_ccw(&mut self) {
        if !self.is_ccw() {
            self.points.reverse();
        }
    }

    pub fn make_cw(&mut self) {
        if self.is_ccw() {
            self.points.reverse();
        }
    }

    /// Ray-casting point-in-polygon test. Boundary cases are not defined (callers
    /// in the slicer test interior points, never points on an edge).
    pub fn contains(&self, p: Point) -> bool {
        let n = self.points.len();
        if n < 3 {
            return false;
        }
        let mut inside = false;
        let mut j = n - 1;
        for i in 0..n {
            let pi = self.points[i];
            let pj = self.points[j];
            if (pi.y > p.y) != (pj.y > p.y) {
                // x of the edge at height p.y (f64 to avoid overflow / int division)
                let t = (p.y - pi.y) as f64 / (pj.y - pi.y) as f64;
                let x_int = pi.x as f64 + t * (pj.x - pi.x) as f64;
                if (p.x as f64) < x_int {
                    inside = !inside;
                }
            }
            j = i;
        }
        inside
    }
}

/// A set of contours forming one slice region: CCW outer boundaries and CW holes.
#[derive(Clone, Debug, Default)]
pub struct Polygons {
    pub contours: Vec<Contour>,
}

impl Polygons {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, c: Contour) {
        self.contours.push(c);
    }

    pub fn is_empty(&self) -> bool {
        self.contours.is_empty()
    }

    /// Net area in mm² (outers minus holes), assuming correct winding.
    pub fn net_area_mm2(&self) -> f64 {
        self.contours.iter().map(|c| c.signed_area_mm2()).sum()
    }

    /// Axis-aligned bounds over all contour points, or `None` if empty.
    pub fn bounds(&self) -> Option<Aabb> {
        let mut pts = self.contours.iter().flat_map(|c| c.points.iter().copied());
        let first = pts.next()?;
        let mut bb = Aabb { min: first, max: first };
        for p in pts {
            bb.expand(p);
        }
        Some(bb)
    }
}

/// Axis-aligned bounding box in Clipper space.
#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub min: Point,
    pub max: Point,
}

impl Aabb {
    pub fn expand(&mut self, p: Point) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
    }

    pub fn union(&mut self, other: &Aabb) {
        self.expand(other.min);
        self.expand(other.max);
    }

    pub fn width(&self) -> Coord {
        self.max.x - self.min.x
    }

    pub fn height(&self) -> Coord {
        self.max.y - self.min.y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn square_area_and_winding() {
        // 10mm CCW square.
        let mut c = Contour::new(vec![
            Point::from_mm(0.0, 0.0),
            Point::from_mm(10.0, 0.0),
            Point::from_mm(10.0, 10.0),
            Point::from_mm(0.0, 10.0),
        ]);
        assert!((c.area_mm2() - 100.0).abs() < 1e-6);
        assert!(c.is_ccw());
        c.make_cw();
        assert!(!c.is_ccw());
        assert!((c.area_mm2() - 100.0).abs() < 1e-6);
    }

    #[test]
    fn contains_interior_point() {
        let c = Contour::new(vec![
            Point::from_mm(0.0, 0.0),
            Point::from_mm(10.0, 0.0),
            Point::from_mm(10.0, 10.0),
            Point::from_mm(0.0, 10.0),
        ]);
        assert!(c.contains(Point::from_mm(5.0, 5.0)));
        assert!(!c.contains(Point::from_mm(15.0, 5.0)));
    }
}
