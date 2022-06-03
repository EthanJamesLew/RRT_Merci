/// basic math methods and math types
use rand::distributions::Uniform;
use rand::{thread_rng, Rng};
use std::mem;

pub type Point2D = (f32, f32);
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

/// given a line segment is start a1 and end a2 and another with start b1 and end b2, determine is they intersect
pub fn line_seg_intersects(a1: &Point2D, a2: &Point2D, b1: &Point2D, b2: &Point2D) -> bool {
    let b = subtract(&a2, &a1);
    let d = subtract(&b2, &b1);
    let bdotd = b.0 * d.1 - b.1 * d.0;

    // maybe float equality?
    if bdotd == 0.0 {
        return false;
    };

    let c = subtract(&b1, &a1);
    let t = (c.0 * d.1 - c.1 * d.0) / bdotd;
    if t < 0.0 || t > 1.0 {
        return false;
    };

    let u = (c.0 * b.1 - c.1 * b.0) / bdotd;
    if u < 0.0 || u > 1.0 {
        return false;
    }

    true 
}


// euclidean distance of point to the origin
pub fn euclidean_distance(pt: &Point2D) -> f32 {
    (pt.0 * pt.0 + pt.1 * pt.1).sqrt()
}


pub fn subtract(pt0: &Point2D, pt1: &Point2D) -> Point2D {
    (pt0.0 - pt1.0, pt0.1 - pt1.1)
}