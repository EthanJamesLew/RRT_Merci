use ncollide2d::shape::{ConvexPolygon};
use ncollide2d::shape::Ball;
use ncollide2d::math::Point;
use ncollide2d::math::Isometry;
use ncollide2d::query::contact_ball_convex_polyhedron;


fn main(){
    let points  = [Point::new(1.0, 1.0), Point::new(2.0, 1.0), Point::new(1.0, 2.0), Point::new(2.0, 2.0)];
    let cp = ConvexPolygon::<f64>::try_from_points(&points).unwrap();

    let prediction = 0.0;
    //let ball_pos = Isometry2::<f64>::new(Vector2::new(1.5, 1.5), zero());
    //let cp_pos = Isometry2::<f64>::new(Vector2::new(0.0, 0.0), zero());
    let ball_pos = Point::new(2.1, 1.5); 
    let cp_pos = Isometry::identity(); 
    let ball = Ball::<f64>::new(0.0);

    let contact = contact_ball_convex_polyhedron(
        &ball_pos, &ball, &cp_pos, &cp, prediction).unwrap();
   
    println!("{:?}", contact.depth);
    for point in cp.points() {
        println!("{:?} {:?}", point.x, point.y);
    }
}