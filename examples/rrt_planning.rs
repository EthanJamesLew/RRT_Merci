/// RRT Example
use rrt_merci as rrt;

fn main() {
    // setup the RRT
    let start = (0.0, 0.0);
    let goal = (10.0, 10.0);
    let expand_dis = 3.0;
    let path_resolution = 0.2;
    let goal_sample_rate = 99;
    let max_iter = 2000;
    let explore_area = rrt::Bounds2D {
        x_min: 0.0,
        x_max: 10.0,
        y_min: 0.0,
        y_max: 10.0,
    };
    let mut rrt = rrt::RRT::new(
        start,
        goal,
        vec![
            rrt::ObstacleSphere {
                x: 5.0,
                y: 5.0,
                radius: 4.5,
            },
            rrt::ObstacleSphere {
                x: 8.0,
                y: 8.0,
                radius: 1.0,
            },
        ],
        expand_dis,
        path_resolution,
        goal_sample_rate,
        max_iter,
        explore_area,
    );

    // get path
    let path = rrt.planning().unwrap();

    for point in path {
        println!("{:?} {:?}", point.0, point.1);
    }

}
