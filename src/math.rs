/// basic math methods and math types
/// TODO: put this in a module?

pub type Point2D = (f32, f32);

// euclidean distance of point to the origin
pub fn euclidean_distance(pt: &Point2D) -> f32 {
    (pt.0 * pt.0 + pt.1 * pt.1).sqrt()
}


pub fn subtract(pt0: &Point2D, pt1: &Point2D) -> Point2D {
    (pt0.0 - pt1.0, pt0.1 - pt1.1)
}

/// given a line segment is start a1 and end a2 and another with start b1 and end b2, determine if they intersect
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
