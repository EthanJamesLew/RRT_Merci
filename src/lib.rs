use rand::distributions::Uniform;
/// RRT Implementation in Rust
/// Make a better way to describe obstacles
/// TODO: implement RRT* and others...
use rand::{rngs::ThreadRng, thread_rng, Rng};

pub struct ObstacleSphere {
    pub x: f32,
    pub y: f32,
    pub radius: f32,
}

/// node of randomly exploring random tree
pub struct RRTNode {
    x: f32,
    y: f32,
    path_x: Vec<f32>,
    path_y: Vec<f32>,
    // FIXME: Rust doesn't like recursive data structures
    //parent: Vec<Option<RRTNode>>
}

/// represent bounds in 2D (inside rectangle)
pub struct Bounds2D {
    pub x_min: f32,
    pub x_max: f32,
    pub y_min: f32,
    pub y_max: f32,
}

/// RRT Configuration Object
pub struct RRT {
    start: (f32, f32),
    goal: (f32, f32),
    obstacles: Vec<ObstacleSphere>,
    expand_dis: f32,
    path_resolution: f32,
    goal_sample_rate: u32,
    max_iter: u32,
    explore_area: Bounds2D,
    //robot_radius: f32,
    node_list: Vec<RRTNode>,
    rng: ThreadRng,
}

impl RRT {
    /// create new "fresh" tree, leave other parameters open
    pub fn new(
        start: (f32, f32),
        goal: (f32, f32),
        obstacles: Vec<ObstacleSphere>,
        expand_dis: f32,
        path_resolution: f32,
        goal_sample_rate: u32,
        max_iter: u32,
        explore_area: Bounds2D,
    ) -> RRT {
        // build it out with defaults on the tree and robot size
        Self {
            start: start,
            goal: goal,
            obstacles: obstacles,
            expand_dis: expand_dis,
            path_resolution: path_resolution,
            goal_sample_rate: goal_sample_rate,
            max_iter: max_iter,
            explore_area: explore_area,
            //robot_radius: 0.0,
            node_list: Vec::<RRTNode>::new(),
            rng: thread_rng(),
        }
    }

    /// RRT Path Planning
    pub fn planning(&mut self) {
        // start by introdcing the start node to the node list
        let start_node = RRTNode {
            x: self.start.0,
            y: self.start.1,
            path_x: Vec::<f32>::new(),
            path_y: Vec::<f32>::new(),
        };
        let end_node = RRTNode {
            x: self.goal.0,
            y: self.goal.1,
            path_x: Vec::<f32>::new(),
            path_y: Vec::<f32>::new(),
        };
        self.node_list.push(start_node);

        // now start the tree search...
        for _idx in 0..self.max_iter {
            let rnd_node = self.get_random_node(&end_node);
            let nearest_ind = self.get_nearest_node_index(&self.node_list, &rnd_node);
            let nearest_node = self
                .node_list
                .get(nearest_ind)
                .expect("RRT Nearest Node failed to get from node list");
            let new_node = self.steer(&nearest_node, &rnd_node, self.expand_dis);

            // do bounds / obstacle checking
            if !self.explore_area.is_outside_area(&new_node) && !self.is_collision(&new_node) {
                println!(
                    "{:?} {:?} new node: {:?} {:?}",
                    self.node_list.len(),
                    nearest_ind,
                    new_node.x,
                    new_node.y
                );
                self.node_list.push(new_node);
            }

            // check if we've reached the goal
            let last_node = self.node_list.last().unwrap();
            if self.calc_distance_to_goal(&last_node) <= self.expand_dis {
                let final_node = self.steer(&last_node, &end_node, self.expand_dis);
                println!(
                    "{:?} {:?} new node: {:?} {:?}",
                    self.node_list.len(),
                    nearest_ind,
                    final_node.x,
                    final_node.y
                );
                self.node_list.push(final_node);
                println!("Goal Reached!");
                break;
            }
            // terminating condition
        }
    }

    fn calc_distance_to_goal(&self, node: &RRTNode) -> f32 {
        let dx = node.x - self.goal.0;
        let dy = node.y - self.goal.1;
        (dx * dx + dy * dy).sqrt()
    }

    fn is_collision(&self, node: &RRTNode) -> bool {
        self.obstacles.iter().any(|obs| {
            ((node.x - obs.x).powf(2.0) + (node.y - obs.y).powf(2.0)).sqrt() <= obs.radius
        })
    }

    /// generate random node (exploration), sometimes going for the goal node (goal sampling rate)
    fn get_random_node(&mut self, end: &RRTNode) -> RRTNode {
        // TODO: make this more efficient with rng setup
        let percent = self.rng.gen_range(0..100);
        if percent > self.goal_sample_rate {
            RRTNode {
                x: end.x,
                y: end.y,
                path_x: Vec::<f32>::new(),
                path_y: Vec::<f32>::new(),
            }
        } else {
            let uniform_x = Uniform::new(self.explore_area.x_min, self.explore_area.x_max);
            let uniform_y = Uniform::new(self.explore_area.y_min, self.explore_area.y_max);
            RRTNode {
                x: self.rng.sample(&uniform_x),
                y: self.rng.sample(&uniform_y),
                path_x: Vec::<f32>::new(),
                path_y: Vec::<f32>::new(),
            }
        }
    }

    /// get nearest node index inside a node_list and target node
    fn get_nearest_node_index(&self, node_list: &Vec<RRTNode>, target_node: &RRTNode) -> usize {
        let mut min_dist = std::f32::MAX;
        let mut min_ind = 0;
        for (idx, node) in node_list.iter().enumerate() {
            let dist = (node.x - target_node.x).powf(2.0) + (node.y - target_node.y).powf(2.0);
            if dist < min_dist {
                min_dist = dist;
                min_ind = idx;
            }
        }
        min_ind
    }

    /// grow out a path from node to node
    fn steer(&self, from_node: &RRTNode, to_node: &RRTNode, expand_dist: f32) -> RRTNode {
        let mut new_node = RRTNode {
            x: from_node.x,
            y: from_node.y,
            path_x: Vec::<f32>::new(),
            path_y: Vec::<f32>::new(),
        };
        new_node.path_x.push(from_node.x);
        new_node.path_y.push(from_node.y);

        let da = self.calc_dist_and_angle(&new_node, &to_node);

        // clip extend length to dist or expand distance
        let extend_length = if expand_dist > da.0 {
            da.0
        } else {
            expand_dist
        };

        // number of times to expand the node (path resolution increments)
        let nexpand = (extend_length / self.path_resolution).floor() as u32;

        // expand out the node
        for _idx in 0..nexpand {
            let new_x = new_node.x + self.path_resolution * da.1.cos();
            let new_y = new_node.y + self.path_resolution * da.1.sin();
            new_node.path_x.push(new_x);
            new_node.path_y.push(new_y);
            new_node.x = new_x;
            new_node.y = new_y;
        }

        // if path is within resolution to the final node, add that
        let d = self.calc_dist_and_angle(&new_node, &to_node);
        if d.0 <= self.path_resolution {
            new_node.path_x.push(to_node.x);
            new_node.path_y.push(to_node.y);
            new_node.x = to_node.x;
            new_node.y = to_node.y;
        }

        new_node
    }

    /// calculate distance and angle between two nodes
    fn calc_dist_and_angle(&self, node_a: &RRTNode, node_b: &RRTNode) -> (f32, f32) {
        let dx = node_b.x - node_a.x;
        let dy = node_b.y - node_a.y;
        let d = (dx.powf(2.0) + dy.powf(2.0)).sqrt();
        let theta = dy.atan2(dx);
        (d, theta)
    }
}

impl Bounds2D {
    /// check if a node is outside the bounds
    pub fn is_outside_area(&self, node: &RRTNode) -> bool {
        node.x < self.x_min || node.x > self.x_max || node.y < self.y_min || node.y > self.y_max
    }
}
