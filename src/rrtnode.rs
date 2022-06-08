/// node of randomly exploring random tree
use crate::math::{euclidean_distance, Point2D};
use serde::{Deserialize, Serialize};


pub trait Node
where
    Self: Sized,
{
    fn new(pt: Point2D) -> Self;

    fn point(&self) -> Point2D;

    fn id(&self) -> usize;
    
    fn parent_id(&self) -> Option<usize>;

    fn get_delta(&self, other_node: &Self) -> (f32, f32);

    /// distance between two nodes
    fn distance_between(&self, other_node: &Self) -> f32 {
        let (dx, dy) = self.get_delta(other_node);
        euclidean_distance(&(dx, dy))
    }

    /// distance between this node and a position tuple
    fn distance_between_pos(&self, pos: (f32, f32)) -> f32;

    /// angle between two nodes
    fn angle_between(&self, other_node: &Self) -> f32 {
        let (dx, dy) = self.get_delta(other_node);
        dy.atan2(dx)
    }

    /// warning: this is slow...
    /// PathTree uses an accelerated data structure
    fn get_nearest_node_index(&self, node_list: &Vec<Self>) -> Option<usize> {
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

/// RRT star is an identified point and path fragment
/// it's parent in the tree is identifed, and a full
/// path can be extracted by traversing the tree
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RRTNode {
    pub id: usize,
    pub parent_id: Option<usize>,
    pub point: Point2D,
    pub path: Vec<Point2D>,
}

/// node for rrt star--normal node + cost
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RRTStarNode {
    pub node: RRTNode,
    pub cost: f32,
}

impl Node for RRTNode {
    /// new node (no parent and no path)
    fn new(pt: Point2D) -> Self {
        Self {
            id: 0,
            parent_id: None,
            point: pt,
            path: Vec::<Point2D>::with_capacity(0),
        }
    }

    fn id(&self) -> usize {
        return self.id
    }
    
    fn parent_id(&self) -> Option<usize> {
        return self.parent_id
    }

    fn point(&self) -> Point2D {
        return (self.point.0, self.point.1)
    }

    fn get_delta(&self, other_node: &Self) -> (f32, f32) {
        let dx = self.point.0 - other_node.point.0;
        let dy = self.point.1 - other_node.point.1;
        return (dx, dy);
    }

    /// distance between this node and a position tuple
    fn distance_between_pos(&self, pos: (f32, f32)) -> f32 {
        let dx = self.point.0 - pos.0;
        let dy = self.point.1 - pos.1;
        euclidean_distance(&(dx, dy))
    }
}

impl Node for RRTStarNode {
    fn new(pt: Point2D) -> Self {
        Self {
            node: RRTNode::new(pt),
            cost: 0.0,
        }
    }
    
    fn point(&self) -> Point2D {
        return self.node.point()
    }
    
    fn id(&self) -> usize {
        return self.node.id()
    }
    
    fn parent_id(&self) -> Option<usize> {
        return self.node.parent_id()
    }

    fn get_delta(&self, other_node: &Self) -> (f32, f32) {
        self.node.get_delta(&other_node.node)
    }

    fn distance_between_pos(&self, pos: (f32, f32)) -> f32 {
        self.node.distance_between_pos(pos)
    }
}
