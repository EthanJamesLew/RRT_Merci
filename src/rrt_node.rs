/// node of randomly exploring random tree

use crate::math::{Point2D, euclidean_distance};

pub struct RRTNode {
    pub id: usize,
    pub parent_id: Option<usize>,
    pub point: Point2D,
    //pub x: f32,
    //pub y: f32,
    pub path: Vec<Point2D>,
    //pub path_y: Vec<f32>,
}


impl RRTNode {
    /// new node (no parent and no path)
    pub fn new(pt: Point2D) -> Self {
        Self {
            id: 0,
            parent_id: None,
            point: pt,
            path: Vec::<Point2D>::with_capacity(0),
        }
    }

    /// distance between two nodes
    pub fn distance_between(&self, other_node: &Self) -> f32 {
        let dx = self.point.0 - other_node.point.0;
        let dy = self.point.1 - other_node.point.1;
        euclidean_distance(&(dx, dy))
    }

    /// distance between this node and a position tuple
    pub fn distance_between_pos(&self, pos: (f32, f32)) -> f32 {
        let dx = self.point.0 - pos.0;
        let dy = self.point.1 - pos.1;
        euclidean_distance(&(dx, dy))
    }

    /// angle between two nodes
    pub fn angle_between(&self, other_node: &Self) -> f32 {
        let dx = self.point.0 - other_node.point.0;
        let dy = self.point.1 - other_node.point.1;
        dy.atan2(dx)
    }
   
    /// given a list of nodes, return the index of the node 
    pub fn get_nearest_node_index(&self, node_list: &Vec<RRTNode>) -> Option<usize> {
        // index on zero sized list is None
        if node_list.len() == 0 {
            return None;
        }

        let mut min_dist = std::f32::MAX;
        let mut min_ind = 0;
        for (idx, node) in node_list.iter().enumerate() {
            let dist = self.distance_between(node);
            if dist < min_dist {
                min_dist = dist;
                min_ind = idx;
            }
        }
        Some(min_ind)
    } 
}
