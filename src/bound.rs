/// 2D boundary objects that have keep out / keep in areas
use crate::math::{Point2D, line_seg_intersects, subtract};

use ncollide2d::shape::{ConvexPolygon};
use ncollide2d::shape::Ball;
use ncollide2d::math::Point;
use ncollide2d::math::Isometry;
use ncollide2d::query::contact_ball_convex_polyhedron;

/// for all collisions, we can determine if a point is inside their area
pub trait Collision {
    fn is_collision(&self, pt: &Point2D) -> bool;
    fn is_collision_segment(&self, start: &Point2D, end: &Point2D) -> bool {
        false
    }
}

/// a simple rectangle described by min / max values
pub struct RectangleBounds {
    pub min_pt: Point2D,
    pub max_pt: Point2D,
}

/// a simple circle with a position 
pub struct CircleBounds {
    pub center_pt: Point2D,
    pub radius: f32,
}

/// convex polygon described from points
pub struct ConvexPolygonBounds {
    pub points: Vec::<Point2D>,
    convex_poly: ConvexPolygon::<f32>
}

impl ConvexPolygonBounds {
    pub fn new_from_points(points: &[Point2D]) -> Option<Self> {
        let points_vec  = points.to_vec();
        let pts =  points_vec.iter().map(|(x, y)| Point::new(*x, *y)).collect::<Vec<Point<f32>>>();
        let cp = ConvexPolygon::<f32>::try_from_points(&pts)?;
        Some(Self{
            points: points_vec,
            convex_poly: cp
        })
    }
    
    pub fn new(points: &Vec<Point2D>) -> Option<Self> {
        let points_vec  = points.to_vec();
        let pts =  points.iter().map(|(x, y)| Point::new(*x, *y)).collect::<Vec<Point<f32>>>();
        let cp = ConvexPolygon::<f32>::try_from_points(&pts)?;
        Some(Self{
            points: points_vec,
            convex_poly: cp
        })
    }
}

impl Collision for ConvexPolygonBounds {
    fn is_collision(&self, pt: &Point2D) -> bool {
        let prediction = 0.0;
        let ball_pos = Point::new(pt.0, pt.1); 
        let cp_pos = Isometry::identity(); 
        let ball = Ball::<f32>::new(0.0);

        let contact = contact_ball_convex_polyhedron(
            &ball_pos, &ball, &cp_pos, &self.convex_poly, prediction);

        contact.is_some()
    }

    fn is_collision_segment(&self, start: &Point2D, end: &Point2D) -> bool {
        // sampling based 
        // FIXME: bad
        let npts = 100;
        let diff = subtract(&end, &start);
        for idx in 0..npts {
            let pt = (start.0 + (idx as f32) * diff.0 / (npts as f32), start.1 + (idx as f32) * diff.1 / (npts as f32));
            if self.is_collision(&pt) {
                return true;
            }
        }
        return false;
    }
}


impl Collision for RectangleBounds {
    fn is_collision(&self, pt: &Point2D) -> bool {
        pt.0 > self.min_pt.0 || pt.0 < self.max_pt.0 || pt.1 > self.min_pt.1 || pt.1 < self.max_pt.1
    }

    fn is_collision_segment(&self, start: &Point2D, end: &Point2D) -> bool {
        let p0 = (self.min_pt.0, self.max_pt.1);
        let p1 = (self.max_pt.0, self.min_pt.1);
        line_seg_intersects(&start, &end, &self.min_pt, &p1) ||
        line_seg_intersects(&start, &end, &self.min_pt, &p0) ||
        line_seg_intersects(&start, &end, &self.max_pt, &p1) ||
        line_seg_intersects(&start, &end, &self.max_pt, &p1)
    }
}

impl Collision for CircleBounds {
    fn is_collision(&self, pt: &Point2D) -> bool {
        let dx = pt.0 - self.center_pt.0;
        let dy = pt.1 - self.center_pt.1;
        (dx * dx + dy * dy) <= (self.radius * self.radius)
    }

    /// line equation collision with obstacles
    fn is_collision_segment(&self, start: &Point2D, end: &Point2D) -> bool {
        let (x1, y1) = start;
        let (x2, y2) = end;

        let a = y2 - y1;
        let b = -(x2 - x1);
        let c = y2 * (x2 - x1) - x2 * (y2 - y1);

        let d = (a * self.center_pt.0 + b * self.center_pt.1 + c).abs() / (a *a + b * b).sqrt();
        d <= self.radius
    }
}