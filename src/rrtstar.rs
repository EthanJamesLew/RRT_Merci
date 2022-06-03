use crate::path::Path2D;
use crate::planner::Planner;
use crate::math::Point2D;
use crate::bound::{Collision, RectangleBounds};
/// Rapidly Exploring Random Tree Star
use crate::rrt::RRT;
use crate::rrtnode::Node;
use crate::rrtnode::{RRTNode, RRTStarNode};

/// two more hyperparameters
pub struct RRTStar<'a> {
    pub rrt: RRT<'a>,
    pub connect_circle_dist: f32,
    pub search_until_max: bool,
    node_list: Vec<RRTStarNode>,
}

/// planner elements of rrtstar
impl Planner<'_> for RRTStar<'_> {
    fn obstacles(&self) -> &Vec<&dyn crate::Collision> {
        self.rrt.obstacles()
    }

    fn plan(&mut self) -> Option<Path2D> {
        // start by introdcing the start node to the node list
        let start_node = RRTNode::new(self.rrt.start);
        let end_node = RRTNode::new(self.rrt.goal);
        self.node_list.push(RRTStarNode {
            node: start_node,
            cost: 0.0,
        });

        // we are building the identifiers to match their position in the array -- this is somewhat fickle,
        // but allows us traverse the tree efficiently without needing to store borrows of the object and
        // manage their lifetimes
        let mut push_idx = 1;
        let mut achieve_goal = false;

        // now start the tree search...
        for _idx in 1..=self.rrt.max_iter {
            let rnd_node = self.get_random_node(&end_node);
            let nearest_ind = rnd_node
                .get_nearest_node_index(&self.node_list)
                .expect("node list should have a size > 0");
            let nearest_node = self
                .node_list
                .get(nearest_ind)
                .expect("RRT Nearest Node failed to get from node list");


            // build out the new RRTStarNode with cost
            let new_node_r = self.rrt.steer(
                &nearest_node.node,
                &rnd_node.node,
                self.rrt.expand_dis,
                push_idx,
            );
            let cost_r = nearest_node.cost + new_node_r.distance_between(&nearest_node.node);
            let new_node = RRTStarNode {
                node: new_node_r,
                cost: cost_r,
            };
            ////println!("nearest node: {:?}", new_node.node.point);

            if !self.is_collision(&new_node.node.point) {

                let near_inds = self.find_near_nodes(&new_node);
                let node_with_updated_parent = self.choose_parent(&new_node, &near_inds, push_idx);
                if node_with_updated_parent.is_some() {
                    let node_p = node_with_updated_parent.unwrap();
                    self.rewire(&node_p, &near_inds);
                    //println!("push rewire");
                    self.node_list.push(node_p);
                    push_idx += 1;
                } else {
                    ////println!("push new");
                    self.node_list.push(new_node);
                    push_idx += 1;
                }
            }

            // TODO: implement the early stopping
        }

        let last_index = self.search_best_goal_node();
        if last_index.is_some() {
            return Some(Path2D(self.get_path(
                self.node_list.get(last_index.unwrap()).unwrap(),
                Vec::<Point2D>::new(),
            )))
        } else {
            return None;
        }
    }
}

impl<'a> RRTStar<'a> {

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
        connect_circle_dist: f32,
        search_until_max: bool,
    ) -> Self {
        // build it out with defaults on the tree and robot size
        let rrt = RRT::new( 
            start,
            goal,
            obstacles,
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
            explore_area,
        );
        RRTStar { rrt: rrt, connect_circle_dist: connect_circle_dist, search_until_max: search_until_max, node_list: Vec::<RRTStarNode>::new() }
    }

    fn find_near_nodes(&self, new_node: &RRTStarNode) -> Vec<usize> {
        let mut near_inds = Vec::new();
        for (idx, node) in self.node_list.iter().enumerate() {
            if new_node.node.distance_between(&node.node) < self.connect_circle_dist {
                near_inds.push(idx);
            }
        }
        near_inds
    }

    fn choose_parent(&self, new_node: &RRTStarNode, near_inds: &Vec<usize>, node_id: usize) -> Option<RRTStarNode> {
        if near_inds.len() == 0 {
            return None;
        }

        let mut costs = Vec::<f32>::new();
        for idx in near_inds {
            let near_node = self.node_list.get(*idx).unwrap();
            let t_node = self.rrt.steer(&near_node.node, &new_node.node, self.rrt.expand_dis, node_id);
            // TODO: check t_node
            if !self.is_collision(&t_node.point) {
                costs.push(self.calc_new_cost(&near_node, &new_node));
            } else {
                costs.push(std::f32::MAX);
            }
        }
        let min_cost = costs.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        if min_cost == f32::INFINITY {
            return None;
        }

        let min_ind = costs.iter().position(|&x| x == min_cost).unwrap();
        let min_node = self.node_list.get(min_ind).unwrap();
        // FIXME: id and parent will be off
        let mut new_node_r = self.rrt.steer(&min_node.node, &new_node.node, self.rrt.expand_dis, node_id);
        let cost_r = min_cost;
        
        Some(RRTStarNode {
            node: new_node_r,
            cost: cost_r,
        })
    }

    fn calc_new_cost(&self, near_node: &RRTStarNode, new_node: &RRTStarNode) -> f32 {
        let new_cost = near_node.cost + new_node.node.distance_between(&near_node.node);
        new_cost
    }

    fn rewire(&mut self, new_node: &RRTStarNode, near_inds: &Vec<usize>) {
        for idx in near_inds {
            //let mut near_node = self.node_list.get(*idx).unwrap();
            let (edge_node, edge_cost, improved_cost) = {
                let edge_node = self.rrt.steer(&new_node.node, &self.node_list[*idx].node, self.rrt.expand_dis, 0);
                // FIXME?
                let edge_cost = self.calc_new_cost(&new_node, &self.node_list[*idx]);
                let improved_cost = self.node_list[*idx].cost > edge_cost;
                (edge_node, edge_cost, improved_cost)
            };

            let no_collision = !self.is_collision(&edge_node.point);

            if no_collision && improved_cost {
                self.propagate_cost_to_leaves(new_node, edge_node, edge_cost, *idx, true);
            }
        }
    }

    fn propagate_cost_to_leaves(&mut self, new_node: &RRTStarNode, edge_node: RRTNode, edge_cost: f32, idx: usize, do_apply: bool) {
        // FIXME: this is a mess
        let is_none = {
            if do_apply {
                self.node_list[idx].node.point = edge_node.point;
                self.node_list[idx].node.path = edge_node.path;
                self.node_list[idx].node.parent_id = edge_node.parent_id;
                self.node_list[idx].cost = edge_cost;
            }
            self.node_list[idx].node.parent_id.is_none()
        };

        for idx in 0..self.node_list.len() {
            if self.node_list[idx].node.parent_id.is_none() {
                continue;
            }
            if self.node_list[idx].node.parent_id.unwrap() == new_node.node.id {
                self.node_list[idx].cost = self.calc_new_cost(&new_node, &self.node_list[idx]);
                let nnode = self.node_list.get(idx).unwrap();
                let node = RRTStarNode { node: RRTNode { id: nnode.node.id, parent_id: nnode.node.parent_id, point: nnode.node.point, path: nnode.node.path.to_vec() }, cost: nnode.cost };
                self.propagate_cost_to_leaves(&node, RRTNode::new((0.0, 0.0)), 0.0, 0, false);
            }
        }
    }
    
    /// get path from an node in the internal node list
    pub fn get_path(&self, goal_node: &RRTStarNode, mut path: Vec<Point2D>) -> Vec<Point2D> {
        path.push(goal_node.node.point);
        match goal_node.node.parent_id {
            None => return path,
            Some(idx) => return self.get_path(self.node_list.get(idx).unwrap(), path),
        }
    }


    fn search_best_goal_node(&self) -> Option<usize> {
        //todo!();
        // TODO: FIXME
        let dist_to_goal = self.node_list.iter().map(|x| x.node.distance_between_pos(self.rrt.goal));
        let mut goal_inds = Vec::<usize>::new();
        for (idx, dist) in dist_to_goal.enumerate() {
            if dist <= self.rrt.expand_dis {
                goal_inds.push(idx);
            }
        }

        let mut safe_goal_inds = Vec::<usize>::new();
        for idx in goal_inds {

            let t_node = self.rrt.steer(&self.node_list.get(idx).unwrap().node, &RRTNode::new(self.rrt.goal), self.rrt.expand_dis, 0);
            if !self.is_collision(&t_node.point) {
                safe_goal_inds.push(idx);
            }
        }

        for node in self.node_list.iter() {
            //println!("{:?}", node.cost);
        }
        let min_cost = safe_goal_inds.iter().fold(f32::INFINITY, |a, &b| a.min(self.node_list[b].cost));
        //println!("min cost {:?}", min_cost);
        for idx in safe_goal_inds {
            if self.node_list[idx].cost == min_cost {
                return Some(idx);
            }
        }

        None
    }

    pub fn get_random_node(&mut self, end: &RRTNode) -> RRTStarNode {
        RRTStarNode {
            node: self.rrt.get_random_node(end),
            cost: 0.0,
        }
    }
}
