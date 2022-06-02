/// RRT Implementation in Rust
/// Make a better way to describe obstacles
/// TODO: implement RRT* and others...
/// TODO: add methods to node (distance between) and obstacles (does collide)
mod rrt_node;
use rrt_node::*;
use rand::distributions::Uniform;
use rand::{rngs::ThreadRng, thread_rng, Rng};

pub struct ObstacleSphere {
    pub x: f32,
    pub y: f32,
    pub radius: f32,
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
    ) -> Self {
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
            node_list: Vec::<RRTNode>::with_capacity(max_iter as usize),
            rng: thread_rng(),
        }
    }

    /// RRT Path Planning
    pub fn planning(&mut self) -> Option<Vec<(f32, f32)>> {
        // start by introdcing the start node to the node list
        let start_node = RRTNode::new(self.start.0, self.start.1);
        let end_node = RRTNode::new(self.goal.0, self.goal.1);        
        self.node_list.push(start_node);

        // we are building the identifiers to match their position in the array -- this is somewhat fickle,
        // but allows us traverse the tree efficiently without needing to store borrows of the object and 
        // manage their lifetimes
        let mut push_idx = 1;
        let mut achieve_goal = false;

        // now start the tree search...
        for _idx in 1..=self.max_iter {
            let rnd_node = self.get_random_node(&end_node);
            let nearest_ind = rnd_node.get_nearest_node_index(&self.node_list)
                .expect("node list should have a size > 0");
            let nearest_node = self
                .node_list
                .get(nearest_ind)
                .expect("RRT Nearest Node failed to get from node list");
            let new_node = self.steer(&nearest_node, &rnd_node, self.expand_dis, push_idx);

            // do bounds / obstacle checking
            if !self.explore_area.is_outside_area(&new_node) && !self.is_collision(&new_node) {
                self.node_list.push(new_node);
                push_idx += 1;
            }

            // check if we've reached the goal
            // terminating condition
            let last_node = self.node_list.last().unwrap();
            if last_node.distance_between_pos(self.goal) <= self.expand_dis {
                let final_node = self.steer(&last_node, &end_node, self.expand_dis, push_idx);
                self.node_list.push(final_node);
                achieve_goal = true;
                break;
            }
        }

        // if goal is met, produce the path
        if achieve_goal {
            Some(self.get_path(self.node_list.last().unwrap(), Vec::<(f32, f32)>::new()))
        } else {
            None
        }

    }

    /// get path from an node in the internal node list
    fn get_path(&self, goal_node: &RRTNode, mut path: Vec<(f32, f32)>) -> Vec<(f32, f32)> {
        path.push((goal_node.x, goal_node.y));
        match goal_node.parent_id {
            None => return path,
            Some(idx) => return self.get_path(self.node_list.get(idx).unwrap(), path)
        }
    }

    /// determine if collision occurs in the obstacle list
    /// maybe move this out to a obstacle struct?
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
            RRTNode::new(end.x, end.y)
        } else {
            let uniform_x = Uniform::new(self.explore_area.x_min, self.explore_area.x_max);
            let uniform_y = Uniform::new(self.explore_area.y_min, self.explore_area.y_max);
            RRTNode::new(self.rng.sample(&uniform_x), self.rng.sample(&uniform_y))
        }
    }

    /// grow out a path from node to node
    fn steer(&self, from_node: &RRTNode, to_node: &RRTNode, expand_dist: f32, index: usize) -> RRTNode {
        let dist = from_node.distance_between(&to_node); //self.calc_dist_and_angle(&from_node, &to_node);
        let theta = to_node.angle_between(&from_node);

        // clip extend length to dist or expand distance
        let extend_length = if expand_dist > dist {
            dist
        } else {
            expand_dist
        };

        // number of times to expand the node (path resolution increments)
        let nexpand = (extend_length / self.path_resolution).floor() as u32;

        let mut new_node = RRTNode {
            id: index,
            parent_id: Some(from_node.id),
            x: from_node.x,
            y: from_node.y,
            path_x: Vec::<f32>::with_capacity(nexpand as usize),
            path_y: Vec::<f32>::with_capacity(nexpand as usize),
        };
        new_node.path_x.push(from_node.x);
        new_node.path_y.push(from_node.y);

        // expand out the node
        for _idx in 0..nexpand {
            let new_x = new_node.x + self.path_resolution * theta.cos();
            let new_y = new_node.y + self.path_resolution * theta.sin();
            new_node.path_x.push(new_x);
            new_node.path_y.push(new_y);
            new_node.x = new_x;
            new_node.y = new_y;
        }

        // if path is within resolution to the final node, add that
        let dist1 = new_node.distance_between(&to_node);
        if dist1 <= self.path_resolution {
            new_node.path_x.push(to_node.x);
            new_node.path_y.push(to_node.y);
            new_node.x = to_node.x;
            new_node.y = to_node.y;
        }

        new_node
    }

}

impl Bounds2D {
    /// check if a node is outside the bounds
    pub fn is_outside_area(&self, node: &RRTNode) -> bool {
        node.x < self.x_min || node.x > self.x_max || node.y < self.y_min || node.y > self.y_max
    }
}
