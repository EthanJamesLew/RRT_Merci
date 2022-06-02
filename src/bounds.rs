/// 2D boundary objects that have keep out / keep in areas
use ncollide2d::shape::{ConvexPolygon};
use ncollide2d::shape::Ball;
use ncollide2d::math::Point;
use ncollide2d::math::Isometry;
use ncollide2d::query::contact_ball_convex_polyhedron;

/// for all collisions, we can determine if a point is inside their area
pub trait Collision {
    fn is_collision(&self, x: f32, y: f32) -> bool;
}

/// a simple rectangle described by min / max values
pub struct RectangleBounds {
    pub x_min: f32,
    pub x_max: f32,
    pub y_min: f32,
    pub y_max: f32,
}

/// a simple circle with a position 
pub struct CircleBounds {
    pub x: f32,
    pub y: f32,
    pub radius: f32,
}

/// convex polygon described from points
pub struct ConvexPolygonBounds {
    pub points: Vec::<(f32, f32)>,
    convex_poly: ConvexPolygon::<f32>
}

impl ConvexPolygonBounds {
    pub fn new_from_points(points: &[(f32, f32)]) -> Option<Self> {
        let points_vec  = points.to_vec();
        let pts =  points_vec.iter().map(|(x, y)| Point::new(*x, *y)).collect::<Vec<Point<f32>>>();
        let cp = ConvexPolygon::<f32>::try_from_points(&pts)?;
        Some(Self{
            points: points_vec,
            convex_poly: cp
        })
    }
    
    pub fn new(points: &Vec<(f32, f32)>) -> Option<Self> {
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
    fn is_collision(&self, x: f32, y: f32) -> bool {
        let prediction = 0.0;
        let ball_pos = Point::new(x, y); 
        let cp_pos = Isometry::identity(); 
        let ball = Ball::<f32>::new(0.0);

        let contact = contact_ball_convex_polyhedron(
            &ball_pos, &ball, &cp_pos, &self.convex_poly, prediction);

        contact.is_some()
    }
}

impl Collision for RectangleBounds {
    fn is_collision(&self, x: f32, y: f32) -> bool {
        x > self.x_min || x < self.x_max || y > self.y_min || y < self.y_max
    }
}

impl Collision for CircleBounds {
    fn is_collision(&self, x: f32, y: f32) -> bool {
        let dx = x - self.x;
        let dy = y - self.y;
        (dx * dx + dy * dy) <= (self.radius * self.radius)
    }
}