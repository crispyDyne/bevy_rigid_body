use core::ops::{Add, Mul, Sub};
use std::ops::{AddAssign, SubAssign};

use nalgebra::{Matrix3, Vector3};

pub type Vector = Vector3<f32>;
pub type Matrix = Matrix3<f32>;

pub fn rx(angle: f32) -> Matrix {
    Matrix::new(
        1.0,
        0.0,
        0.0,
        0.0,
        angle.cos(),
        angle.sin(),
        0.0,
        -angle.sin(),
        angle.cos(),
    )
}

pub fn ry(angle: f32) -> Matrix {
    Matrix::new(
        angle.cos(),
        0.0,
        -angle.sin(),
        0.0,
        1.0,
        0.0,
        angle.sin(),
        0.0,
        angle.cos(),
    )
}

pub fn rz(angle: f32) -> Matrix {
    Matrix::new(
        angle.cos(),
        angle.sin(),
        0.0,
        -angle.sin(),
        angle.cos(),
        0.0,
        0.0,
        0.0,
        1.0,
    )
}

#[derive(Debug, Copy, Clone)]
pub struct Velocity {
    pub vel: Vector,
}

#[derive(Debug, Copy, Clone)]
pub struct Xform {
    pub position: Vector,
    pub rotation: Matrix,
}

impl Default for Xform {
    fn default() -> Self {
        Self::identity()
    }
}

impl Xform {
    pub fn new(position: Vector, rotation: Matrix) -> Self {
        Self { position, rotation }
    }
    pub fn identity() -> Self {
        Self {
            position: Vector::zeros(),
            rotation: Matrix::identity(),
        }
    }
    pub fn inverse(self) -> Self {
        Self {
            position: -(self.rotation * self.position),
            rotation: self.rotation.transpose(),
        }
    }
    pub fn rotx(angle: f32) -> Self {
        Self {
            rotation: rx(angle),
            ..Default::default()
        }
    }
    pub fn roty(angle: f32) -> Self {
        Self {
            rotation: ry(angle),
            ..Default::default()
        }
    }
    pub fn rotz(angle: f32) -> Self {
        Self {
            rotation: rz(angle),
            ..Default::default()
        }
    }
    pub fn posx(x: f32) -> Self {
        Self {
            position: Vector::new(x, 0.0, 0.0),
            ..Default::default()
        }
    }
    pub fn posy(y: f32) -> Self {
        Self {
            position: Vector::new(0.0, y, 0.0),
            ..Default::default()
        }
    }
    pub fn posz(z: f32) -> Self {
        Self {
            position: Vector::new(0.0, 0.0, z),
            ..Default::default()
        }
    }

    pub fn transform_point(self, point: Vector) -> Vector {
        self.rotation * (point - self.position)
    }
}

impl Mul<Xform> for Xform {
    type Output = Xform;

    fn mul(self, rhs: Xform) -> Xform {
        Xform {
            position: rhs.position + rhs.rotation.transpose() * self.position,
            rotation: self.rotation * rhs.rotation,
        }
    }
}

impl Mul<Motion> for Xform {
    type Output = Motion;

    fn mul(self, rhs: Motion) -> Motion {
        Motion {
            v: self.rotation * (rhs.v - self.position.cross(&rhs.w)),
            w: self.rotation * rhs.w,
        }
    }
}

impl Mul<Force> for Xform {
    type Output = Force;

    fn mul(self, rhs: Force) -> Force {
        Force {
            f: self.rotation * rhs.f,
            m: self.rotation * (rhs.m - self.position.cross(&rhs.f)),
        }
    }
}

impl Mul<Vector> for Xform {
    type Output = Vector;

    fn mul(self, rhs: Vector) -> Vector {
        self.rotation * rhs
    }
}

impl Mul<Velocity> for Xform {
    type Output = Velocity;

    fn mul(self, rhs: Velocity) -> Velocity {
        Velocity {
            vel: self.rotation * rhs.vel,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Motion {
    pub v: Vector,
    pub w: Vector,
}

impl Motion {
    pub fn new(v_data: [f32; 3], w_data: [f32; 3]) -> Self {
        Self {
            v: Vector::new(v_data[0], v_data[1], v_data[2]),
            w: Vector::new(w_data[0], w_data[1], w_data[2]),
        }
    }
    pub fn zero() -> Self {
        Self {
            v: Vector::zeros(),
            w: Vector::zeros(),
        }
    }

    pub fn cross_v(self, rhs: Motion) -> Motion {
        Motion {
            v: self.w.cross(&rhs.v) + self.v.cross(&rhs.w),
            w: self.w.cross(&rhs.w),
        }
    }

    pub fn cross_f(self, rhs: Force) -> Force {
        Force {
            f: self.w.cross(&rhs.f),
            m: self.w.cross(&rhs.m) + self.v.cross(&rhs.f),
        }
    }

    pub fn velocity_point(self, point: Vector) -> Velocity {
        Velocity {
            vel: self.w.cross(&point) + self.v,
        }
    }
}

impl Add<Motion> for Motion {
    type Output = Motion;
    fn add(self, rhs: Motion) -> Motion {
        Motion {
            v: self.v + rhs.v,
            w: self.w + rhs.w,
        }
    }
}

impl Default for Motion {
    fn default() -> Self {
        Self::zero()
    }
}

impl Mul<Motion> for f32 {
    type Output = Motion;
    fn mul(self, rhs: Motion) -> Motion {
        Motion {
            w: self * rhs.w,
            v: self * rhs.v,
        }
    }
}

// impl Mul<Vector> for Motion {
//     type Output = Velocity;
//     fn mul(self, rhs: Vector) -> Velocity {
//         Velocity {
//             vel: self.w.cross(&rhs) + self.v,
//         }
//     }
// }

#[derive(Debug, Copy, Clone)]
pub struct Force {
    pub f: Vector,
    pub m: Vector,
}

impl Force {
    pub fn new(f_data: [f32; 3], m_data: [f32; 3]) -> Self {
        Self {
            f: Vector::new(f_data[0], f_data[1], f_data[2]),
            m: Vector::new(m_data[0], m_data[1], m_data[2]),
        }
    }
    pub fn zero() -> Self {
        Self {
            f: Vector::zeros(),
            m: Vector::zeros(),
        }
    }

    pub fn self_outer_product(self) -> InertiaAB {
        InertiaAB {
            m: self.f * self.f.transpose(),
            c: self.m * self.f.transpose(),
            moi: self.m * self.m.transpose(),
        }
    }

    pub fn force_point(force: Vector, point: Vector) -> Force {
        Force {
            f: force,
            m: point.cross(&force),
        }
    }
}

impl Default for Force {
    fn default() -> Self {
        Self::zero()
    }
}

impl Add<Force> for Force {
    type Output = Force;
    fn add(self, rhs: Force) -> Force {
        Force {
            m: self.m + rhs.m,
            f: self.f + rhs.f,
        }
    }
}

impl AddAssign<Force> for Force {
    fn add_assign(&mut self, rhs: Force) {
        self.m += rhs.m;
        self.f += rhs.f;
    }
}

impl Mul<Force> for f32 {
    type Output = Force;
    fn mul(self, rhs: Force) -> Force {
        Force {
            m: self * rhs.m,
            f: self * rhs.f,
        }
    }
}

impl Sub for Force {
    type Output = Force;
    fn sub(self, rhs: Force) -> Force {
        Force {
            m: self.m - rhs.m,
            f: self.f - rhs.f,
        }
    }
}

impl SubAssign<Force> for Force {
    fn sub_assign(&mut self, rhs: Force) {
        self.m -= rhs.m;
        self.f -= rhs.f;
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct Inertia {
    m: f32,
    c: Vector,
    moi: Matrix,
}

impl Inertia {
    pub fn new(m: f32, c: Vector, moi: Matrix) -> Inertia {
        Inertia { m, c, moi }
    }
    pub fn zero() -> Inertia {
        Inertia {
            m: 0.0,
            c: Vector::zeros(),
            moi: Matrix::zeros(),
        }
    }
}

impl Mul<Motion> for Inertia {
    type Output = Force;
    fn mul(self, rhs: Motion) -> Force {
        Force {
            f: self.m * rhs.v - self.c.cross(&rhs.w),
            m: self.moi * rhs.w + self.c.cross(&rhs.v),
        }
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct InertiaAB {
    m: Matrix,
    c: Matrix,
    moi: Matrix,
}

impl From<Inertia> for InertiaAB {
    fn from(i: Inertia) -> Self {
        let c_cross = i.c.cross_matrix();
        InertiaAB {
            m: i.m * Matrix::identity(),
            c: i.m * c_cross,
            moi: i.moi - i.m * c_cross * c_cross,
        }
    }
}

impl Mul<Motion> for InertiaAB {
    type Output = Force;
    fn mul(self, rhs: Motion) -> Force {
        Force {
            f: self.m * rhs.v + self.c.transpose() * rhs.w,
            m: self.moi * rhs.w + self.c * rhs.v,
        }
    }
}

impl Mul<InertiaAB> for f32 {
    type Output = InertiaAB;
    fn mul(self, rhs: InertiaAB) -> InertiaAB {
        InertiaAB {
            c: self * rhs.c,
            m: self * rhs.m,
            moi: self * rhs.moi,
        }
    }
}

impl Add<InertiaAB> for InertiaAB {
    type Output = InertiaAB;
    fn add(self, rhs: InertiaAB) -> InertiaAB {
        InertiaAB {
            c: self.c + rhs.c,
            m: self.m + rhs.m,
            moi: self.moi + rhs.moi,
        }
    }
}

impl AddAssign<InertiaAB> for InertiaAB {
    fn add_assign(&mut self, rhs: InertiaAB) {
        self.c += rhs.c;
        self.m += rhs.m;
        self.moi += rhs.moi;
    }
}

impl Sub<InertiaAB> for InertiaAB {
    type Output = InertiaAB;
    fn sub(self, rhs: InertiaAB) -> InertiaAB {
        InertiaAB {
            c: self.c - rhs.c,
            m: self.m - rhs.m,
            moi: self.moi - rhs.moi,
        }
    }
}

impl Mul<InertiaAB> for Xform {
    type Output = InertiaAB;

    fn mul(self, inertia: InertiaAB) -> InertiaAB {
        let rot_m = self.rotation;
        let i_cross = self.position.cross_matrix();

        InertiaAB {
            // definitely not tested
            m: rot_m * inertia.m * rot_m.transpose(),
            c: rot_m * (inertia.c - (i_cross * inertia.m)) * rot_m.transpose(),
            moi: rot_m
                * ((inertia.moi - (i_cross * inertia.c.transpose()))
                    + ((inertia.c - (i_cross * inertia.m)) * i_cross))
                * rot_m.transpose(),
        }
    }
}
