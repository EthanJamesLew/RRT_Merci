/// RRT Example
use rrt_merci as rrt;

fn main() {
    // setup the RRT
    let start = (0.0, 0.0);
    let goal = (10.0, 0.0);
    let expand_dis = 1.0;
    let path_resolution = 1.0;
    let goal_sample_rate = 50;
    let max_iter = 100000;
    let explore_area = rrt::RectangleBounds {
        x_min: 0.0,
        x_max: 10.0,
        y_min: 0.0,
        y_max: 10.0,
    };
    
    // obstacle 1 is a simple circle
    let o1 = rrt::CircleBounds {
                x: 5.0,
                y: 5.0,
                radius: 2.5,
    };

    // obstacle 2 is a convex polygon
    let points = vec![(3.0, -1.0), (8.0, 9.0), (3.0, 9.0), (8.0, -1.0)];
    let o2 = rrt::ConvexPolygonBounds::new(&points).expect("cannot make a convex polygon from points");
    
    let mut rrt = rrt::RRT::new(
        start,
        goal,
        vec![
            &o1,
            &o2,
        ],
        expand_dis,
        path_resolution,
        goal_sample_rate,
        max_iter,
        explore_area,
    );

    // get path
    let path = rrt.planning().expect("path not found!");

    for point in path {
        println!("{:?} {:?}", point.0, point.1);
    }

}
