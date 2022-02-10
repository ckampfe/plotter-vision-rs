use nalgebra::{Matrix4, Vector4};

use crate::Vector;

fn m44_mult(a: Matrix4<f32>, b: Matrix4<f32>) -> Matrix4<f32> {
    // let mut c: Matrix4<f32> = Matrix4::zeros();

    // for i in 0..4 {
    //     for j in 0..4 {
    //         for k in 0..4 {
    //             c[(i, j)] += a[(i, k)] * b[(k, j)]
    //         }
    //     }
    // }

    // c
    a * b
}

struct Camera {
    eye: Vector,
    lookat: Vector,
    up: Vector,
    fov: f32,
    generation: usize,
    width: f32,
    height: f32,
    matrix: Matrix4<f32>,
    u: Vector,
    v: Vector,
    w: Vector,
}

impl Camera {
    fn project(&self, v_in: Vector, v_out: Option<Vector>) -> Option<Vector> {
        let v = Vector4::new(v_in.x, v_in.y, v_in.z, 1.0f32);
        let mut p = Vector4::new(0.0, 0.0, 0.0, 0.0);

        for i in 0..4 {
            for j in 0..4 {
                p[i] += self.matrix[(i, j)] * v[j];
            }
        }

        if p[2] <= 0.0 {
            return None;
        }

        let x = p[0] / p[3];
        let y = p[1] / p[3];
        let z = p[2] / p[3];
        if v_out.is_none() {
            return Some(Vector::new(x, y, z));
        }

        v_out.unwrap().x = x;
        v_out.unwrap().y = y;
        v_out.unwrap().z = z;

        v_out
    }

    fn update_matrix(&mut self) {
        // Update the camera projection matrix with eye/lookat/fov
        // compute the three basis vectors for the camera

        // w is the Z axis from the eye to the destination point
        let w = (self.eye - self.lookat).normalize();

        // u is the X axis to the right side of the camera
        let u = self.up.cross(&w).normalize();

        // v is the Y axis aligned with the UP axis
        let v = w.cross(&u).normalize();

        let cam = Matrix4::new(
            u.x,
            u.y,
            u.z,
            -u.dot(&self.eye),
            v.x,
            v.y,
            v.z,
            -v.dot(&self.eye),
            w.x,
            w.y,
            w.z,
            -w.dot(&self.eye),
            0.0,
            0.0,
            0.0,
            1.0,
        );

        let scale = 1000.0 / (self.fov * std::f32::consts::PI / 180.0 / 2.0).tan();
        let near = 1.0;
        let far = 200.0;
        let f1 = -far / (far - near);
        let f2 = -far * near / (far - near);

        let perspective = Matrix4::new(
            scale, 0.0, 0.0, 0.0, 0.0, scale, 0.0, 0.0, 0.0, 0.0, f2, -1.0, 0.0, 0.0, f1, 0.0,
        );

        self.matrix = m44_mult(perspective, cam);
        self.u = u;
        self.v = v;
        self.w = w;

        self.generation += 1;
    }
}
