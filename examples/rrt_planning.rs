/// RRT Example
use rrt_merci as rrt;
use rrt_merci::Planner;
use rrt_merci::RRTStar;

fn main() {
    // setup the RRT
    let start = (0.0, 0.0);
    let goal = (6.0, 9.5);
    let expand_dis = 0.4;
    let path_resolution = 0.2;
    let goal_sample_rate = 95;
    let max_iter = 5000;
    let explore_area = rrt::RectangleBounds {
        min_pt: (0.0, 0.0),
        max_pt: (12.0, 10.0),
    };

    // obstacle 2 is a convex polygon
    //let points0 = vec![(0.0, 3.0), (2.0, 3.0), (2.0, 5.0), (0.0, 5.0)];
    //let points1 = vec![(4.0, 9.0), (4.0, -1.0), (6.0, -1.0), (6.0, 9.0)];
    //let points2 = vec![(7.0, 5.0), (7.0, 7.0), (12.0, 5.0), (12.0, 7.0)];
    //let o0 = rrt::ConvexPolygonBounds::new(&points0).expect("cannot make a convex polygon from points");
    //let o1 = rrt::ConvexPolygonBounds::new(&points1).expect("cannot make a convex polygon from points");
    //let o2 = rrt::ConvexPolygonBounds::new(&points2).expect("cannot make a convex polygon from points");

    let o0 = rrt::RectangleBounds {
        min_pt: (0.0, 3.0),
        max_pt: (2.0, 5.0),
    };
    let o1 = rrt::RectangleBounds {
        min_pt: (4.0, -1.0),
        max_pt: (6.0, 9.0),
    };
    let o2 = rrt::RectangleBounds {
        min_pt: (7.0, 5.0),
        max_pt: (12.0, 7.0),
    };
    let o3 = rrt::RectangleBounds {
        min_pt: (6.0, 2.0),
        max_pt: (11.0, 3.0),
    };

    //let mut rrt = rrt::RRT::new(
    //    start,
    //    goal,
    //    vec![&o0, &o1, &o2, &o3],
    //    expand_dis,
    //    path_resolution,
    //    goal_sample_rate,
    //    max_iter,
    //    explore_area,
    //);

    let mut rrt = RRTStar::new(
        start,
        goal,
        vec![&o0, &o1, &o2, &o3],
        expand_dis,
        path_resolution,
        goal_sample_rate,
        max_iter,
        explore_area,
        100000.0,
        true,
    );

    // get path
    let path_res = rrt.plan().expect("path not found!");
    let path = path_res.path_smoothing_obstacle(&rrt.rrt.obstacles, 1000);

    println!("{:?}", rrt.node_list);
    //for point in path.0 {
    //    println!("{:?} {:?}", point.0, point.1);
    //}
}
