use pyo3::{exceptions::PyRuntimeError, prelude::*};
use rrt_merci::{bound as rbound, rrt, Collision, Planner};

trait Collider {
    fn get_collider(&self) -> &dyn Collision;
}

/// Circle Boundary
/// --
///
/// Represents a circle in 2D space by a center point and a radius.
#[pyclass]
struct CircleBounds(rbound::CircleBounds);

/// Convex Polygon Boundary
/// --
///
/// Represents a convex polygon in 2D space by a set of points. Not all
/// set of points can be used.
#[pyclass]
struct ConvexPolygonBounds(rbound::ConvexPolygonBounds);

#[pymethods]
impl CircleBounds {
    #[new]
    fn new(x: f32, y: f32, radius: f32) -> Self {
        Self(rbound::CircleBounds {
            center_pt: (x, y),
            radius: radius,
        })
    }

    fn is_collision(&self, points: Vec<(f32, f32)>) -> Vec<bool> {
        points.iter().map(|pt| self.0.is_collision(pt)).collect()
    }

    fn is_collision_segment(&self, points: Vec<((f32, f32), (f32, f32))>) -> Vec<bool> {
        points
            .iter()
            .map(|(pt0, pt1)| self.0.is_collision_segment(pt0, pt1))
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "<Circle: center ({:?}, {:?}) radius {:?}>",
            self.0.center_pt.0, self.0.center_pt.1, self.0.radius
        )
    }
}

impl Collider for CircleBounds {
    fn get_collider(&self) -> &dyn Collision {
        &self.0
    }
}

#[pymethods]
impl ConvexPolygonBounds {
    #[new]
    fn new(pts: Vec<(f32, f32)>) -> PyResult<Self> {
        match rbound::ConvexPolygonBounds::new(&pts) {
            Some(e) => Ok(Self(e)),
            None => Err(PyRuntimeError::new_err(
                "cannot build convex polygon from points",
            )),
        }
    }

    fn is_collision(&self, points: Vec<(f32, f32)>) -> Vec<bool> {
        points.iter().map(|pt| self.0.is_collision(pt)).collect()
    }

    fn is_collision_segment(&self, points: Vec<((f32, f32), (f32, f32))>) -> Vec<bool> {
        points
            .iter()
            .map(|(pt0, pt1)| self.0.is_collision_segment(pt0, pt1))
            .collect()
    }

    fn __repr__(&self) -> String {
        format!("<ConvexPolygon: points <{:?}>>", self.0.points)
    }
}

impl Collider for ConvexPolygonBounds {
    fn get_collider(&self) -> &dyn Collision {
        &self.0
    }
}

#[pyclass]
struct TreePlannerResult {
    node_tree: Vec<rrt_merci::RRTNode>,
    path: Option<Vec<(f32, f32)>>,
    smooth_path: Option<Vec<(f32, f32)>>,
}

#[pyfunction]
fn plan_rrt(
    start: (f32, f32),
    goal: (f32, f32),
    obstacles: Vec<(f32, f32, f32)>,
    expand_dis: f32,
    path_resolution: f32,
    goal_sample_rate: u32,
    max_iter: u32,
    explore_area: ((f32, f32), (f32, f32)),
) -> Option<Vec<(f32, f32)>> {
    let mut new_obstacles = Vec::<&dyn Collision>::new();
    let new_sphere: Vec<rbound::CircleBounds> = obstacles
        .iter()
        .map(|(x, y, r)| rbound::CircleBounds {
            center_pt: (*x, *y),
            radius: *r,
        })
        .collect();
    for oi in new_sphere.iter() {
        new_obstacles.push(oi);
    }
    let new_explore_area = rbound::RectangleBounds {
        min_pt: explore_area.0,
        max_pt: explore_area.1,
    };
    let mut rrt = rrt::RRT::new(
        start,
        goal,
        new_obstacles,
        expand_dis,
        path_resolution,
        goal_sample_rate,
        max_iter,
        new_explore_area,
    );
    let path_res = rrt.plan();
    match path_res {
        Some(p) => {
            //let path = p.path_smoothing_obstacle(&rrt.obstacles, 1000);

            //let node_tree: Vec<RRTNode> = rrt.node_list.iter().map(|n| n.node.clone()).collect();

            Some(p.0)
            //TreePlannerResult {
            //    node_tree: node_tree,
            //    smooth_path: Some(path.0),
            //    path: Some(p.0),
            //}
        }
        None => {
            None
            //TreePlannerResult {
            //    node_tree: rrt.node_list,
            //    smooth_path: None,
            //    path: None
            //}
        }
    }
}

/// A Python module implemented in Rust.
#[pymodule]
fn mercipy(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<CircleBounds>()?;
    m.add_class::<ConvexPolygonBounds>()?;
    m.add_class::<TreePlannerResult>()?;
    m.add_function(wrap_pyfunction!(plan_rrt, m)?)?;
    Ok(())
}
