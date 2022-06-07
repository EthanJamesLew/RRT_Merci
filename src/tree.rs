
use kiddo::{KdTree, distance::squared_euclidean};
use crate::math::Point2D;
use std::collections::BTreeMap;

use crate::RRTNode;

/// branching path tree
/// --
///
/// a spatial data structure that stores branching paths
/// (e.g. the ones used in RRT)
pub struct PathTree {
    kd_tree: KdTree<f32, usize, 2>,
    b_map: BTreeMap<usize, RRTNode>
}

impl PathTree {
    pub fn new() -> Self {
        Self{
            kd_tree: KdTree::<f32, usize, 2>::new(),
            b_map: BTreeMap::<usize, RRTNode>::new()
        }
    }

    pub fn add_node(&mut self, node: RRTNode) {
        let res = self.kd_tree.add(&[node.point.0, node.point.1], node.id);
        // TODO: handle this better
        match res {
            Ok(_) => {
                self.b_map.insert(node.id, node);
            },
            Err(_) => return
        };
    }

    pub fn get(&self, index: usize) -> Option<&RRTNode> {
        self.b_map.get(&index)
    }

    /// get the index id for the closest node in the tree
    /// --
    /// 
    /// this is accelerated via a spatial data structure (kiddo KD-Tree)
    pub fn get_nearest_node_index(&self, node: &RRTNode) -> Option<usize> {
        let res = self.kd_tree.nearest_one(&[node.point.0, node.point.1], &squared_euclidean);
        match res {
            Ok(s) => {
                Some(*s.1)
            },
            Err(_) => None
        }
    }

    /// get a specific path from the branching paths
    /// --
    /// 
    /// given a node, traverse its parents up to the root node and return a path (vector of points)
    pub fn get_path(&self, goal_node: &RRTNode, mut path: Vec<Point2D>) -> Vec<Point2D> {
        path.push(goal_node.point);
        match goal_node.parent_id {
            None => return path,
            Some(idx) => return self.get_path(self.get(idx).unwrap(), path),
        }
    }

    /// get the last entry added
    pub fn last(&self) -> Option<&RRTNode> {
        let e = self.b_map.keys().last();
        match e {
            None => return None,
            Some(v) => self.b_map.get(v)
        }
    }

}