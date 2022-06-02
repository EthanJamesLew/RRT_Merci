/// node of randomly exploring random tree
pub struct RRTNode {
    pub id: usize,
    pub parent_id: Option<usize>,
    pub x: f32,
    pub y: f32,
    pub path_x: Vec<f32>,
    pub path_y: Vec<f32>,
}


impl RRTNode {
    /// new node (no parent and no path)
    pub fn new(x: f32, y: f32) -> Self {
        Self {
            id: 0,
            parent_id: None,
            x: x,
            y: y,
            path_x: Vec::<f32>::with_capacity(0),
            path_y: Vec::<f32>::with_capacity(0)
        }
    }

    /// distance between two nodes
    pub fn distance_between(&self, other_node: &Self) -> f32 {
        let dx = self.x - other_node.x;
        let dy = self.y - other_node.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// distance between this node and a position tuple
    pub fn distance_between_pos(&self, pos: (f32, f32)) -> f32 {
        let dx = self.x - pos.0;
        let dy = self.y - pos.1;
        (dx * dx + dy * dy).sqrt()
    }

    /// angle between two nodes
    pub fn angle_between(&self, other_node: &Self) -> f32 {
        let dx = self.x - other_node.x;
        let dy = self.y - other_node.y;
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
