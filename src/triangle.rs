use crate::{Point, Vector};

#[derive(Clone, Debug)]
pub struct Triangle {
    pub is_invisible: bool,
    pub min: Vector,
    pub max: Vector,
    pub t1: Vector,
    pub t2: Vector,
    pub screen: [Vector; 3],
}

impl Triangle {
    pub(crate) fn bary_coord(&self, p: &Point) -> Vector {
        let t1 = self.t1;
        let t2 = self.t2;
        let px = p.x - self.screen[0].x;
        let py = p.y - self.screen[0].y;

        let d = t1.x * t2.y - t2.x * t1.y;
        let a = (px * t2.y - py * t2.x) / d;
        let b = (py * t1.x - px * t1.y) / d;

        Vector::new(a, b, self.screen[0].z + a * t1.z + b * t2.z)
    }
}
