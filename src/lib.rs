#![crate_type = "lib"]

/// node of randomly exploring random tree
pub struct RRTNode{
    x: f32,
    y: f32,
    path_x: Vec<f32>,
    path_y: Vec<f32>,
    parent: Vec<Option<RRTNode>>
}

/// represent bounds in 2D (inside rectangle)
pub struct Bounds2D {
    x_min: f32,
    x_max: f32,
    y_min: f32,
    y_max: f32
}

/// RRT Configuration Object
pub struct RRT{
    start: (f32, f32),
    goal: (f32, f32),
    expand_dis: f32,
    path_resolution: f32,
    goal_sample_rate: u32,
    max_iter: u32,
    explore_area: Bounds2D,
    robot_radius: f32,
    node_list: Vec<RRTNode>,
}


impl RRT {
    /// create new "fresh" tree, leave other parameters open
    pub fn new(
        start: (f32, f32),
        goal: (f32, f32),
        expand_dis: f32,
        path_resolution: f32,
        goal_sample_rate: u32,
        max_iter: u32,
        explore_area: Bounds2D
    ) -> RRT {
        Self{
            start: start,
            goal: goal,
            expand_dis: expand_dis,
            path_resolution: path_resolution,
            goal_sample_rate: goal_sample_rate,
            max_iter: max_iter,
            explore_area: explore_area,
            robot_radius: 0.0,
            node_list: Vec::<RRTNode>::new()
        }
    }
}
