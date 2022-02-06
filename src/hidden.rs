use crate::triangle::Triangle;
use std::collections::{HashMap, VecDeque};

const EPSILON: f32 = 0.0000001;
const STL_KEY2D_SCALE: usize = 16;

type Vector = nalgebra::Vector3<f32>;
type Point = nalgebra::Point3<f32>;
type WorkQueue = VecDeque<Segment>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Occlude {
    NoOcclusion,
    InFront,
    Hidden,
    Clipped,
    Split,
}

#[derive(Clone, Copy, Debug)]
pub struct Segment {
    p0: Point,
    p1: Point,
}

fn occlude(t: &Triangle, s: &mut Segment, work_queue: &mut WorkQueue) -> Occlude {
    let mut p_max = Vector::default();
    let mut p_min = Vector::default();

    if t.is_invisible {
        return Occlude::NoOcclusion;
    }

    let seg_len = dist2(&s.p1, &s.p0);

    if seg_len < 1.0 {
        return Occlude::Hidden;
    }

    v3max(&mut p_max, &s.p0, &s.p1);
    v3min(&mut p_min, &s.p0, &s.p1);

    if p_max.z <= t.min.z {
        return Occlude::InFront;
    }

    if p_max.x < t.min.x || t.max.x < p_min.x {
        return Occlude::NoOcclusion;
    }
    if p_max.y < t.min.y || t.max.y < p_min.y {
        return Occlude::NoOcclusion;
    }

    let tp0 = t.bary_coord(&s.p0);
    let tp1 = t.bary_coord(&s.p1);

    if inside(&tp0.into()) && inside(&tp1.into()) {
        if s.p0.z < tp0.z + EPSILON && s.p1.z < tp1.z + EPSILON {
            return Occlude::NoOcclusion;
        }

        // this segment either punctures the triangle
        // or something is bad about it.  ignore the other cases
        return Occlude::Hidden;
    }

    let mut intercept_s = vec![];
    let mut intercept_t = vec![];
    let mut intercepts = 0;

    for i in 0..3 {
        let intercept = intercept_lines(
            &s.p0,
            &s.p1,
            &t.screen[i].into(),
            &t.screen[(i + 1) % 3].into(),
        );
        match intercept {
            Intercept::No => continue,
            Intercept::V([is, it]) => {
                // if the segment intercept is closer than the triangle
                // intercept, then this does not count as an intersection
                if is.z <= it.z {
                    continue;
                }

                intercepts += 1;
                intercept_s.push(is);
                intercept_t.push(it);
            }
        }
    }

    // let original_intercepts = intercepts;
    if intercepts == 0 {
        return Occlude::NoOcclusion;
    }

    if intercepts == 3 {
        if close_enough(&intercept_s[0], &intercept_s[2])
            || close_enough(&intercept_s[1], &intercept_s[2])
        {
            intercepts -= 1;
        } else if close_enough(&intercept_s[0], &intercept_s[1]) {
            intercept_s[1] = intercept_s[2];
            intercept_t[1] = intercept_t[2];
            intercepts -= 1;
        } else {
            // this should never happen, unless there are very small triangles
            // in which case we discard this triangle
            return Occlude::Hidden;
        }
    }

    if intercepts == 2 && close_enough(&intercept_s[0], &intercept_s[1]) {
        intercepts -= 1;
    }

    if intercepts == 1 {
        if inside(&tp0.into()) {
            // clipped from is0 to p1
            s.p0 = intercept_s[0];
            return Occlude::Clipped;
        }

        if inside(&tp1.into()) {
            // clipped from p0 to is0
            s.p1 = intercept_s[0];
            return Occlude::Clipped;
        }

        // this might be a tangent, so nothing is clipped
        // return tri_no_occlusion;
        return Occlude::NoOcclusion;
    }

    let d00 = dist2(&intercept_s[0], &s.p0);
    let d01 = dist2(&intercept_s[1], &s.p0);
    let d10 = dist2(&intercept_s[0], &s.p1);
    let d11 = dist2(&intercept_s[1], &s.p1);

    if d00 < EPSILON && d11 < EPSILON {
        return Occlude::Hidden;
    }
    if d01 < EPSILON && d10 < EPSILON {
        return Occlude::Hidden;
    }

    if d00 < EPSILON {
        s.p0 = intercept_s[1];
        return Occlude::Clipped;
    } else if d01 < EPSILON {
        s.p0 = intercept_s[0];
        return Occlude::Clipped;
    } else if d10 < EPSILON {
        s.p1 = intercept_s[1];
        return Occlude::Clipped;
    } else if d11 < EPSILON {
        s.p1 = intercept_s[0];
        return Occlude::Clipped;
    }

    // neither end point matches, so we'll create a new
    // segment that excludes the space between is0 and is1
    let midpoint = if d00 > d01 { 1 } else { 0 };

    work_queue.push_back(Segment {
        p0: s.p0,
        p1: intercept_s[midpoint],
    });

    s.p0 = intercept_s[if midpoint == 1 { 0 } else { 1 }];

    Occlude::Split
}

// Process a segment against a list of triangles
// returns a list of segments that are visible
// TODO: best if triangles is sorted by z depth
// TODO: figure out a better representation for the screen map
pub fn hidden_wire(
    s: &mut Segment,
    screen_map: &mut HashMap<String, Vec<Triangle>>,
    work_queue: &mut WorkQueue,
) -> Option<Segment> {
    let min_key_x: usize = (s.p0.x.min(s.p1.x).trunc() / STL_KEY2D_SCALE as f32).trunc() as usize;
    let min_key_y: usize = (s.p0.y.min(s.p1.y).trunc() / STL_KEY2D_SCALE as f32).trunc() as usize;
    let max_key_x: usize = (s.p0.x.max(s.p1.x) / STL_KEY2D_SCALE as f32).trunc() as usize;
    let max_key_y: usize = (s.p0.y.max(s.p1.y) / STL_KEY2D_SCALE as f32).trunc() as usize;

    let mut triangles = vec![];

    // for(let x = min_key_x ; x <= max_key_x ; x++)
    for x in min_key_x..=max_key_x {
        // for(let y = min_key_y ; y <= max_key_y ; y++)
        for y in min_key_y..=max_key_y {
            triangles = screen_map[&format!("{x},{y}")].clone();
            // if (!triangles) {
            if triangles.is_empty() {
                continue;
            }
        }

        // for(let t of triangles)
        for t in &triangles {
            if t.is_invisible {
                continue;
            }

            let rc = occlude(t, s, work_queue);

            // this segment is no longer visible,
            // but any new segments that it has added to the array
            // will be processed against the triangles again.
            if rc == Occlude::Hidden {
                return None;
            }

            if rc == Occlude::InFront {
                // this line segment is entirely in front of
                // this triangle, which means that no other
                // triangles on the sorted list can occlude
                // the segment, so we're done.
                break;
            }

            if rc == Occlude::Clipped || rc == Occlude::NoOcclusion {
                continue;
            }

            if rc == Occlude::Split {
                continue;
            }
        }
    }

    // if we have made it all the way here, the remaining part
    // of this segment is visible and should be added to the draw list
    Some(*s)
}

fn dist2(p0: &Point, p1: &Point) -> f32 {
    let dx = p0.x - p1.x;
    let dy = p0.y - p1.y;
    dx * dx + dy * dy
}

fn v3max(out: &mut Vector, v0: &Point, v1: &Point) {
    out.x = v0.x.max(v1.x);
    out.y = v0.y.max(v1.y);
    out.z = v0.z.max(v1.z);
}

fn v3min(out: &mut Vector, v0: &Point, v1: &Point) {
    out.x = v0.x.min(v1.x);
    out.y = v0.y.min(v1.y);
    out.z = v0.z.min(v1.z);
}

fn inside(pb: &Point) -> bool {
    let a = pb.x;
    let b = pb.y;
    -EPSILON <= a && -EPSILON <= b && a + b <= 1.0 + EPSILON
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Intercept {
    No,
    V([Point; 2]),
}

fn intercept_lines(p0: &Point, p1: &Point, p2: &Point, p3: &Point) -> Intercept {
    let s0 = p1 - p0;
    let s1 = p3 - p2;

    // compute s0 x s1
    let d = s0.x * s1.y - s1.x * s0.y;

    // if they are close to parallel then we define that
    // as non-intersecting
    if -EPSILON < d && d < EPSILON {
        // return [-1,null,null];
        return Intercept::No;
    }

    // compute how far along each line they would intersect
    let r0: f32 = (s1.x * (p0.y - p2.y) - s1.y * (p0.x - p2.x)) / d;
    let r1: f32 = (s0.x * (p0.y - p2.y) - s0.y * (p0.x - p2.x)) / d;

    // if they are outside of (0,1) then the intersection
    // occurs outside of either segment and are non-intersecting
    if !(0.0..=1.0).contains(&r0) || !(0.0..=1.0).contains(&r1) {
        return Intercept::No;
    }

    // compute the points of intersections for the two
    // segments as p + r * s
    let s0 = p0 + (s0 * r0);

    let s1 = p2 + s1 * r1;

    Intercept::V([s0, s1])
}

fn close_enough(p0: &Point, p1: &Point) -> bool {
    let eps = 0.0001;

    let dx = p0.x - p1.x;
    if dx < -eps || eps < dx {
        return false;
    }

    let dy = p0.y - p1.y;
    if dy < -eps || eps < dy {
        return false;
    }

    let dz = p0.z - p1.z;
    if dz < -eps || eps < dz {
        return false;
    }

    true
}
