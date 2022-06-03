use crate::path::Path2D;
use crate::planner::Planner;
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

            if self.is_collision(&new_node.node.point) {
                let near_inds = self.find_near_nodes(&new_node);
                let node_with_updated_parent = self.choose_parent(&new_node, &near_inds);
                if node_with_updated_parent.is_some() {
                    let node_p = node_with_updated_parent.unwrap();
                    self.rewire(&node_p, &near_inds);
                    self.node_list.push(node_p);
                    push_idx += 1;
                } else {
                    self.node_list.push(new_node);
                    push_idx += 1;
                }
            }

            // TODO: implement the early stopping
        }

        let last_index = self.search_best_goal_node();
        if last_index.is_some() {
            return self.generate_final_course(last_index);
        } else {
            return None;
        }
    }
}

impl<'a> RRTStar<'a> {
    fn find_near_nodes(&self, new_node: &RRTStarNode) -> Vec<usize> {
        let mut near_inds = Vec::new();
        for (idx, node) in self.node_list.iter().enumerate() {
            if new_node.node.distance_between(&node.node) < self.connect_circle_dist {
                near_inds.push(idx);
            }
        }
        near_inds
    }

    fn choose_parent(&self, new_node: &RRTStarNode, near_inds: &Vec<usize>) -> Option<RRTStarNode> {
        todo!();
    }

    fn rewire(&mut self, new_node: &RRTStarNode, near_inds: &Vec<usize>) {
        todo!();
    }

    fn search_best_goal_node(&self) -> Option<usize> {
        todo!();
    }

    fn generate_final_course(&self, last_index: Option<usize>) -> Option<Path2D> {
        todo!();
    }

    pub fn get_random_node(&mut self, end: &RRTNode) -> RRTStarNode {
        RRTStarNode {
            node: self.rrt.get_random_node(end),
            cost: 0.0,
        }
    }
}
