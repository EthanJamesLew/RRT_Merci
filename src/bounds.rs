/// 2D boundary objects that have keep out / keep in areas

pub trait Collision {
    fn is_collision(&self, x: f32, y: f32) -> bool;
}

pub struct RectangleBounds {
    pub x_min: f32,
    pub x_max: f32,
    pub y_min: f32,
    pub y_max: f32,
}

pub struct CircleBounds {
    pub x: f32,
    pub y: f32,
    pub radius: f32,
}

impl Collision for RectangleBounds {
    fn is_collision(&self, x: f32, y: f32) -> bool {
        x > self.x_min || x < self.x_max || y > self.y_min || y < self.y_max
    }
}

impl Collision for CircleBounds {
    fn is_collision(&self, x: f32, y: f32) -> bool {
        let dx = x - self.x;
        let dy = y - self.y;
        (dx * dx + dy * dy) <= (self.radius * self.radius)
    }
}