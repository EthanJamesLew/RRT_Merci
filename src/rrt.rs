/// Rapidly Exploring Random Trees (Simple)
use crate::bound::*;
use crate::math::*;
use crate::path::Path2D;
use crate::planner::Planner;
use crate::rrtnode::Node;
use crate::rrtnode::RRTNode;
use rand::distributions::Uniform;
use rand::{rngs::ThreadRng, thread_rng, Rng};

/// RRT Configuration Object
pub struct RRT<'a> {
    pub start: Point2D,
    pub goal: Point2D,
    pub obstacles: Vec<&'a dyn Collision>,
    pub expand_dis: f32,
    pub path_resolution: f32,
    pub goal_sample_rate: u32,
    pub max_iter: u32,
    pub explore_area: RectangleBounds,
    pub node_list: Vec<RRTNode>,
    rng: ThreadRng,
}

impl Planner<'_> for RRT<'_> {
    /// give access to the obstacle list
    fn obstacles(&self) -> &Vec<&dyn Collision> {
        &self.obstacles
    }

    /// RRT Path Planning
    fn plan(&mut self) -> Option<Path2D> {
        // start by introdcing the start node to the node list
        let start_node = RRTNode::new(self.start);
        let end_node = RRTNode::new(self.goal);
        self.node_list.push(start_node);

        // we are building the identifiers to match their position in the array -- this is somewhat fickle,
        // but allows us traverse the tree efficiently without needing to store borrows of the object and
        // manage their lifetimes
        let mut push_idx = 1;
        let mut achieve_goal = false;

        // now start the tree search...
        for _idx in 1..=self.max_iter {
            let rnd_node = self.get_random_node(&end_node);
            let nearest_ind = rnd_node
                .get_nearest_node_index(&self.node_list)
                .expect("node list should have a size > 0");
            let nearest_node = self
                .node_list
                .get(nearest_ind)
                .expect("RRT Nearest Node failed to get from node list");
            let new_node = self.steer(&nearest_node, &rnd_node, self.expand_dis, push_idx);

            // do bounds / obstacle checking
            let edge_collision_occured = {
                match new_node.parent_id {
                    None => false,
                    Some(parent_id) => {
                        let parent_node = self
                            .node_list
                            .get(parent_id as usize)
                            .expect("RRT Parent Node failed to get from node list");
                        self.is_collision_segment(&parent_node.point, &new_node.point)
                    }
                }
            };
            if self.explore_area.is_collision(&new_node.point)
                && !self.is_collision(&new_node.point)
                && !edge_collision_occured
            {
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
            Some(Path2D(self.get_path(
                self.node_list.last().unwrap(),
                Vec::<Point2D>::new(),
            )))
        } else {
            None
        }
    }
}

impl<'a> RRT<'a> {
    /// create new tree, leave other parameters open
    pub fn new(
        start: Point2D,
        goal: Point2D,
        obstacles: Vec<&'a dyn Collision>,
        expand_dis: f32,
        path_resolution: f32,
        goal_sample_rate: u32,
        max_iter: u32,
        explore_area: RectangleBounds,
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

    /// get path from an node in the internal node list
    pub fn get_path(&self, goal_node: &RRTNode, mut path: Vec<Point2D>) -> Vec<Point2D> {
        path.push(goal_node.point);
        match goal_node.parent_id {
            None => return path,
            Some(idx) => return self.get_path(self.node_list.get(idx).unwrap(), path),
        }
    }

    /// generate random node (exploration), sometimes going for the goal node (goal sampling rate)
    pub fn get_random_node(&mut self, end: &RRTNode) -> RRTNode {
        // TODO: make this more efficient with rng setup
        let percent = self.rng.gen_range(0..100);
        if percent > self.goal_sample_rate {
            RRTNode::new(end.point)
        } else {
            let uniform_x = Uniform::new(self.explore_area.min_pt.0, self.explore_area.max_pt.0);
            let uniform_y = Uniform::new(self.explore_area.min_pt.1, self.explore_area.max_pt.1);
            RRTNode::new((self.rng.sample(&uniform_x), self.rng.sample(&uniform_y)))
        }
    }

    /// grow out a path from node to node
    pub fn steer(
        &self,
        from_node: &RRTNode,
        to_node: &RRTNode,
        expand_dist: f32,
        index: usize,
    ) -> RRTNode {
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
            point: from_node.point,
            path: Vec::<Point2D>::with_capacity(nexpand as usize),
        };
        new_node.path.push(from_node.point);

        // expand out the node
        for _idx in 0..nexpand {
            let new_x = new_node.point.0 + self.path_resolution * theta.cos();
            let new_y = new_node.point.1 + self.path_resolution * theta.sin();
            new_node.path.push((new_x, new_y));
            new_node.point = (new_x, new_y);
        }

        // if path is within resolution to the final node, add that
        let dist1 = new_node.distance_between(&to_node);
        if dist1 <= self.path_resolution {
            new_node.path.push(to_node.point);
            new_node.point = to_node.point;
        }

        new_node
    }
}
