use crate::bound::Collision;
/// path implementation (trace of points)
use crate::math::{euclidean_distance, subtract, Point2D};
use rand::distributions::Uniform;
use rand::{thread_rng, Rng};
use serde::{Deserialize, Serialize};

use std::mem;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Path2D(pub Vec<Point2D>);

impl Path2D {
    pub fn path_length(&self) -> f32 {
        let mut dist = 0.0;

        if self.0.len() == 0 {
            return dist;
        }

        for idx in 0..(self.0.len() - 1) {
            let p0 = self.0.get(idx + 1).unwrap();
            let p1 = self.0.get(idx).unwrap();
            let dp = subtract(p0, p1);
            dist += euclidean_distance(&dp);
        }

        dist
    }

    pub fn get_target_point(&self, target: f32) -> (Point2D, usize) {
        let mut le = 0.0;
        let mut ti: usize = 0;
        let mut last_length = 0.0;

        for idx in 1..(self.0.len() - 1) {
            let p0 = self.0.get(idx + 1).unwrap();
            let p1 = self.0.get(idx).unwrap();
            let dp = subtract(p0, p1);
            let d = euclidean_distance(&dp);
            le += d;

            if le >= target {
                ti = idx - 1;
                last_length = d;
                break;
            }
        }

        let part_ratio = (le - target) / last_length;

        let p0 = self.0.get(ti).unwrap();
        let p1 = self.0.get(ti + 1).unwrap();
        let x = p0.0 + (p1.0 - p0.0) * part_ratio;
        let y = p0.1 + (p1.1 - p0.1) * part_ratio;

        ((x, y), ti)
    }

    /// obstacle aware random sampling smoothing
    pub fn path_smoothing_obstacle(&self, obstacles: &Vec<&dyn Collision>, max_iter: u32) -> Self {
        let mut rng = thread_rng();
        let mut path = Path2D(self.0.to_vec());

        for _idx in 0..max_iter {
            let le = path.path_length();
            let uniform = Uniform::new(0.0, le);

            let mut p0 = rng.sample(&uniform);
            let mut p1 = rng.sample(&uniform);

            if p1 < p0 {
                mem::swap(&mut p0, &mut p1);
            }

            let first = path.get_target_point(p0);
            let second = path.get_target_point(p1);

            if first.1 <= 0 || second.1 <= 0 {
                continue;
            }

            if (second.1 + 1) > path.0.len() {
                continue;
            }

            if second.1 == first.1 {
                continue;
            }

            let mut any_collisions = false;
            for obs in obstacles.iter() {
                if obs.is_collision_segment(&first.0, &second.0) {
                    any_collisions = true;
                    break;
                }
            }
            if any_collisions {
                continue;
            }

            let mut new_path = Path2D(Vec::<Point2D>::new());
            for idx in 0..(first.1 + 1) {
                let val = path.0.get(idx).unwrap();
                new_path.0.push((val.0, val.1));
            }
            new_path.0.push(first.0);
            new_path.0.push(second.0);
            for idx in (second.1 + 1)..path.0.len() {
                let val = path.0.get(idx).unwrap();
                new_path.0.push((val.0, val.1));
            }
            path = new_path;
        }

        path
    }
}
