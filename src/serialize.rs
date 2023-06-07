use std::fmt;

use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct ModelDef {
    pub joints: Vec<JointDef>,
    pub name: String,
    pub systems: Vec<SystemDef>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum JointTypeDef {
    Base,
    Px,
    Py,
    Pz,
    Rx,
    Ry,
    Rz,
}

impl fmt::Display for JointTypeDef {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}
#[derive(Debug, Serialize, Deserialize)]
pub struct JointDef {
    pub name: String,
    pub joint_type: JointTypeDef,
    pub parent: Option<String>,
    pub transform: TransformDef,
    pub inertia: InertiaDef,
    pub meshes: Vec<MeshDef>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct MeshDef {
    pub mesh_type: MeshTypeDef,
    pub transform: TransformDef,
    pub color: [f32; 4],
}
#[derive(Debug, Serialize, Deserialize)]
pub enum MeshTypeDef {
    Box { half_extents: [f32; 3] },
    Cylinder { height: f32, radius: f32 },
    // Sphere { radius: f32 },
    // Mesh { filename: String },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransformDef {
    pub position: [f32; 3],
    pub quaternion: [f32; 4],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InertiaDef {
    pub mass: f32,
    pub center_of_mass: [f32; 3],
    pub inertia: [f32; 6],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemDef {
    pub system_type: SystemTypeDef,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SystemTypeDef {
    Steering(SteeringDef),
    Drive(DrivenWheelDef),
    Brake(BrakeWheelDef),
    Suspension(SuspensionDef),
    TireContact(TireContactDef),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SteeringDef {
    pub joint: String,
    pub max_angle: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrivenWheelDef {
    pub joint: String,
    pub max_torque: f32,
    pub max_speed: f32,
    pub max_power: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BrakeWheelDef {
    pub joint: String,
    pub max_torque: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SuspensionDef {
    pub joint: String,
    pub stiffness: f32,
    pub damping: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TireContactDef {
    pub joint: String,
    pub radius: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub longitudinal_stiffness: f32,
    pub lateral_stiffness: f32,
}
