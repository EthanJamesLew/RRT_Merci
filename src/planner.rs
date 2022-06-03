use crate::bound::*;
use crate::math::*;
use crate::path::Path2D;

pub trait Planner<'a> {
    /// planners are obstacle aware
    fn obstacles(&self) -> &Vec<&dyn Collision>;

    /// determine if collision occurs in the obstacle list
    /// maybe move this out to a obstacle struct?
    fn is_collision(&self, point: &Point2D) -> bool {
        if self.obstacles().len() == 0 {
            return false;
        }
        self.obstacles().iter().any(|obs| obs.is_collision(&point))
    }

    /// planners may or may not return a path (a path may not exist, or the implementation isn't able to find one)
    fn plan(&mut self) -> Option<Path2D>;
}
