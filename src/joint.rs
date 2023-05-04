use bevy::prelude::*;
use bevy_integrator::integrator::Stateful;
use std::ops::{Add, Mul};

use crate::mesh::Mesh as RBDA_Mesh;
use crate::serialize::{JointDef, JointTypeDef};
use crate::sva::{Force, Inertia, InertiaAB, Motion, Xform};

#[derive(Default, Debug)]
pub enum JointType {
    Base,
    #[default]
    Rx,
    Ry,
    Rz,
    Px,
    Py,
    Pz,
}

#[derive(Component, Default, Debug)]
pub struct Base;

#[derive(Component, Default, Debug)]
pub struct Joint {
    pub joint_type: JointType,
    pub name: String,

    // joint definition
    pub s: Motion,
    pub i: Inertia,
    pub xt: Xform,

    // joint state (and solution)
    pub q: f32,
    pub qd: f32,
    pub qdd: f32,

    // common parameters
    pub xl: Xform,
    pub xj: Xform,
    pub x: Xform,
    pub v: Motion,
    pub vj: Motion,
    pub c: Motion,
    pub a: Motion,

    // algorithm specific parameters
    pub iaa: InertiaAB,
    pub paa: Force,
    pub tau: f32,
    pub f_ext: Force,
    pub dd: f32,
    pub u: f32,
    pub uu: Force,
    pub meshes: Vec<RBDA_Mesh>,
}

impl Joint {
    pub fn base(a: Motion) -> Self {
        Self {
            a,
            joint_type: JointType::Base,
            ..Default::default()
        }
    }
    pub fn rx(name: String, inertia: Inertia, xt: Xform) -> Self {
        let s = Motion::new([0., 0., 0.], [1., 0., 0.]);

        Self {
            name,
            i: inertia,
            xt,
            s,
            joint_type: JointType::Rx,
            ..Default::default()
        }
    }

    pub fn ry(name: String, inertia: Inertia, xt: Xform) -> Self {
        let s = Motion::new([0., 0., 0.], [0., 1., 0.]);

        Self {
            name,
            i: inertia,
            xt,
            s,
            joint_type: JointType::Ry,
            ..Default::default()
        }
    }

    pub fn rz(name: String, inertia: Inertia, xt: Xform) -> Self {
        let s = Motion::new([0., 0., 0.], [0., 0., 1.]);

        Self {
            name,
            i: inertia,
            xt,
            s,
            joint_type: JointType::Rz,
            ..Default::default()
        }
    }
    pub fn px(name: String, inertia: Inertia, xt: Xform) -> Self {
        let s = Motion::new([1., 0., 0.], [0., 0., 0.]);

        Self {
            name,
            i: inertia,
            xt,
            s,
            joint_type: JointType::Px,
            ..Default::default()
        }
    }
    pub fn py(name: String, inertia: Inertia, xt: Xform) -> Self {
        let s = Motion::new([0., 1., 0.], [0., 0., 0.]);

        Self {
            name,
            i: inertia,
            xt,
            s,
            joint_type: JointType::Py,
            ..Default::default()
        }
    }
    pub fn pz(name: String, inertia: Inertia, xt: Xform) -> Self {
        let s = Motion::new([0., 0., 1.], [0., 0., 0.]);

        Self {
            name,
            i: inertia,
            xt,
            s,
            joint_type: JointType::Pz,
            ..Default::default()
        }
    }

    pub fn from_joint_def(joint_def: &JointDef) -> Self {
        let i = Inertia::from_def(&joint_def.inertia);
        let xt = Xform::from_def(&joint_def.transform);

        let (s, joint_type) = match joint_def.joint_type {
            JointTypeDef::Base => (Motion::new([0., 0., 0.], [0., 0., 0.]), JointType::Base),
            JointTypeDef::Rx => (Motion::new([0., 0., 0.], [1., 0., 0.]), JointType::Rx),
            JointTypeDef::Ry => (Motion::new([0., 0., 0.], [0., 1., 0.]), JointType::Ry),
            JointTypeDef::Rz => (Motion::new([0., 0., 0.], [0., 0., 1.]), JointType::Rz),
            JointTypeDef::Px => (Motion::new([1., 0., 0.], [0., 0., 0.]), JointType::Px),
            JointTypeDef::Py => (Motion::new([0., 1., 0.], [0., 0., 0.]), JointType::Py),
            JointTypeDef::Pz => (Motion::new([0., 0., 1.], [0., 0., 0.]), JointType::Pz),
        };

        Self {
            i,
            xt,
            s,
            joint_type,
            ..Default::default()
        }
    }
}

pub fn bevy_joint_positions(mut joint_transform_query: Query<(&mut Joint, &mut Transform)>) {
    for (joint, mut transform) in joint_transform_query.iter_mut() {
        transform.translation = Vec3::from_slice(joint.xl.position.data.as_slice());
        let mat = Mat3::from_cols_slice(joint.xl.rotation.data.as_slice()).transpose();
        transform.rotation = Quat::from_mat3(&mat);
    }
}

impl Into<f32> for JointState {
    fn into(self) -> f32 {
        self.q
    }
}

impl Stateful for Joint {
    type State = JointState;
    fn get_state(&self) -> Self::State {
        Self::State {
            q: self.q,
            qd: self.qd,
        }
    }

    fn set_state(&mut self, state: &Self::State) {
        self.q = state.q;
        self.qd = state.qd;
    }

    fn get_dstate(&self) -> Self::State {
        Self::State {
            q: self.qd,
            qd: self.qdd,
        }
    }

    fn set_dstate(&mut self, dstate: Self::State) {
        self.qd = dstate.q;
        self.qdd = dstate.qd;
    }

    fn reset(&mut self) {
        self.qdd = 0.;
        self.f_ext = Force::zero();
        self.tau = 0.;
    }

    fn get_name(&self) -> String {
        self.name.clone()
    }
}

#[derive(Clone)]
pub struct JointState {
    pub q: f32,
    pub qd: f32,
}

impl JointState {
    pub fn new(q: f32, qd: f32) -> Self {
        Self { q, qd }
    }
    pub fn zero() -> Self {
        Self::new(0., 0.)
    }
    pub fn from_joint(joint: &Joint) -> Self {
        Self::new(joint.q, joint.qd)
    }
}

impl Add for JointState {
    type Output = JointState;
    fn add(self, other: JointState) -> JointState {
        JointState {
            q: self.q + other.q,
            qd: self.qd + other.qd,
        }
    }
}

impl Mul<f32> for JointState {
    type Output = JointState;
    fn mul(self, other: f32) -> JointState {
        JointState {
            q: self.q * other,
            qd: self.qd * other,
        }
    }
}
