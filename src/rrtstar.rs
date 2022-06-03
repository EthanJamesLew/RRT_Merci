use crate::planner::Planner;
/// Rapidly Exploring Random Tree Star
use crate::rrt::RRT;
use crate::rrtnode::{RRTNode, RRTStarNode};

/// two more hyperparameters
pub struct RRTStar<'a> {
    pub rrt: RRT<'a>,
    pub connect_circle_dist: f32,
    pub search_until_max: bool,
}

/// planner elements of rrtstar
impl Planner<'_> for RRTStar<'_> {
    fn obstacles(&self) -> &Vec<&dyn crate::Collision> {
        self.rrt.obstacles()
    }

    fn plan(&mut self) -> Option<crate::path::Path2D> {
        todo!();
    }
}
