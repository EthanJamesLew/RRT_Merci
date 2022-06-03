
/// path implementation (trace of points)
use crate::math::{euclidean_distance, subtract, Point2D};

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
       let mut ti : usize = 0;
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


}
