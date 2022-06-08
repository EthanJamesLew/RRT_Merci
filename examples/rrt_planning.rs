use std::any::Any;

use rrt::Node;
use rrt::RRTStarNode;
use rrt::path::Path2D;
use rrt::RRTNode;
/// RRT Example
use rrt_merci as rrt;
use rrt_merci::math::Point2D;
use rrt_merci::Planner;
use rrt_merci::RRTStar;
use serde::{Deserialize, Serialize};
use serde_json::json;

#[derive(Serialize, Deserialize)]
struct RRTReturn {
    tree: Vec<RRTNode>,
    path: Path2D,
    smooth_path: Path2D,
}

fn main() {
    // setup the RRT
    let start = (0.0, 0.0);
    let goal = (8.0, 0.0);
    let expand_dis = 0.2;
    let path_resolution = 0.2;
    let goal_sample_rate = 0;
    let max_iter = 20000;
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
        1000000.0,
        false,
    );

    // get path
    let path_res = rrt.plan().expect("path not found!");
    let path = path_res.path_smoothing_obstacle(&rrt.rrt.obstacles, 1000);

    //let node_tree: Vec<RRTNode> = rrt.node_list.iter().map(|n| n.node.clone()).collect();
    let node_tree: Vec<RRTNode> = rrt.node_tree.node_list()
        .iter()
        .map(|v| RRTNode{
            id: v.id(),
            parent_id: v.parent_id(),
            point: v.point(),
            path: v.node.path.to_vec()
        }).collect();


    let ret = RRTReturn {
        tree: node_tree,
        smooth_path: path,
        path: path_res,
    };

    let ret_json = serde_json::json!(ret);
    println!("{}", ret_json.to_string());
}
