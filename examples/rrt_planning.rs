use rrt_merci as rrt;

fn main() {
    let start = (0.0, 0.0);
    let goal = (10.0, 10.0);
    let expand_dis = 1.0;
    let path_resolution = 0.1;
    let goal_sample_rate = 50;
    let max_iter = 100;
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
                radius: 1.0,
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
    rrt.planning();
}
