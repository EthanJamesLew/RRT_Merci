/// RRT Example
use rrt_merci as rrt;

fn main() {
    // setup the RRT
    let start = (0.0, 0.0);
    let goal = (12.0, 0.0);
    let expand_dis = 0.3;
    let path_resolution = 0.2;
    let goal_sample_rate = 50;
    let max_iter = 200000;
    let explore_area = rrt::RectangleBounds {
        min_pt: (0.0, 0.0),
        max_pt: (12.0, 10.0)
    };
    
    // obstacle 1 is a simple circle
    let o1 = rrt::CircleBounds {
        center_pt: (5.0, 5.0),        
        radius: 2.5,
    };

    // obstacle 2 is a convex polygon
    let points0 = vec![(0.0, 3.0), (2.0, 3.0), (2.0, 5.0), (0.0, 5.0)];
    let points1 = vec![(4.0, 9.0), (4.0, -1.0), (6.0, -1.0), (6.0, 9.0)];
    let points2 = vec![(7.0, 5.0), (7.0, 7.0), (12.0, 5.0), (12.0, 7.0)];
    let o0 = rrt::ConvexPolygonBounds::new(&points0).expect("cannot make a convex polygon from points");
    let o1 = rrt::ConvexPolygonBounds::new(&points1).expect("cannot make a convex polygon from points");
    let o2 = rrt::ConvexPolygonBounds::new(&points2).expect("cannot make a convex polygon from points");
    
    let mut rrt = rrt::RRT::new(
        start,
        goal,
        vec![
            &o0,
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
    let path_res = rrt.planning().expect("path not found!");
    let path = rrt.path_smoothing(&path_res, 1000);

    for point in path.0 {
        println!("{:?} {:?}", point.0, point.1);
    }

}
