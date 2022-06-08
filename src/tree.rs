
use kiddo::{KdTree, distance::squared_euclidean};
use crate::{math::Point2D, Node};
use std::collections::BTreeMap;

use crate::RRTNode;

/// branching path tree
/// --
///
/// a spatial data structure that stores branching paths
/// (e.g. the ones used in RRT)
pub struct PathTree<T> where T: Node {
    kd_tree: KdTree<f32, usize, 2>,
    pub b_map: BTreeMap<usize, T>
}

impl<T> PathTree<T> where T: Node{
    pub fn new() -> Self {
        Self{
            kd_tree: KdTree::<f32, usize, 2>::new(),
            b_map: BTreeMap::<usize, T>::new()
        }
    }

    pub fn add_node(&mut self, node: T) {
        let point = node.point();
        let res = self.kd_tree.add(&[point.0, point.1], node.id());
        // TODO: handle this better
        match res {
            Ok(_) => {
                self.b_map.insert(node.id(), node);
            },
            Err(_) => return
        };
    }

    pub fn get(&self, index: usize) -> Option<&T> {
        self.b_map.get(&index)
    }

    /// get the index id for the closest node in the tree
    /// --
    /// 
    /// this is accelerated via a spatial data structure (kiddo KD-Tree)
    pub fn get_nearest_node_index(&self, node: &T) -> Option<usize> {
        let point = node.point();
        let res = self.kd_tree.nearest_one(&[point.0, point.1], &squared_euclidean);
        match res {
            Ok(s) => {
                Some(*s.1)
            },
            Err(_) => None
        }
    }

    /// indices of nodes within r-ball of a node
    pub fn get_within(&self, node: &T, radius: f32) -> Vec<usize> {
        let point = node.point();
        let r = self.kd_tree.within(&[point.0, point.1], radius, &squared_euclidean);
        match r {
            Ok(s) => {
                let v: Vec<usize> = s.iter().map(
                    |(d, idx)| **idx
                ).collect();
                v
            },
            Err(_) => Vec::<usize>::new()
        }
    }

    /// set an existing node in the data structure
    pub fn set(&mut self, node: T) {
        // TODO: error handling
        let id = node.id();
        let point = node.point();
        if self.b_map.contains_key(&id) {
            let old_id = self.b_map.get(&id).unwrap().id();
            self.kd_tree.remove(&[point.0, point.1], &old_id);

        }
        self.b_map.remove(&node.id());
        self.b_map.insert(node.id(), node);
        self.kd_tree.add(&[point.0, point.1], id);
    }

    /// get a specific path from the branching paths
    /// --
    /// 
    /// given a node, traverse its parents up to the root node and return a path (vector of points)
    pub fn get_path(&self, goal_node: &T, mut path: Vec<Point2D>) -> Vec<Point2D> {
        path.push(goal_node.point());
        match goal_node.parent_id() {
            None => return path,
            Some(idx) => return self.get_path(self.get(idx).unwrap(), path),
        }
    }

    /// get the last entry added
    pub fn last(&self) -> Option<&T> {
        let e = self.b_map.keys().last();
        match e {
            None => return None,
            Some(v) => self.b_map.get(v)
        }
    }

    /// get the tree length (number of nodes in the tree)
    pub fn len(&self) -> usize {
        self.b_map.len()
    }

    /// get a list of node references
    pub fn node_list(&self) -> Vec<&T> {
        let r: Vec<&T> = self.b_map.iter().map(|(k, v)| v).collect();
        r
    }

}