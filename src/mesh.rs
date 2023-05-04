use crate::serialize::{MeshDef, MeshTypeDef};
use bevy::prelude::{shape, Mesh as BevyMesh};

#[derive(Debug)]
pub struct BoxMesh {
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
    pub min_z: f32,
    pub max_z: f32,
}

impl BoxMesh {
    pub fn new(min_x: f32, max_x: f32, min_y: f32, max_y: f32, min_z: f32, max_z: f32) -> Self {
        Self {
            min_x,
            max_x,
            min_y,
            max_y,
            min_z,
            max_z,
        }
    }
    pub fn to_bevy_mesh(self) -> BevyMesh {
        BevyMesh::from(shape::Box {
            max_x: self.max_x,
            min_x: self.min_x,
            max_y: self.max_y,
            min_y: self.min_y,
            max_z: self.max_z,
            min_z: self.min_z,
        })
    }
}

#[derive(Debug)]
pub enum Mesh {
    Box(BoxMesh),
    // Cylinder,
}

impl Mesh {
    pub fn from_mesh_def(mesh_def: &MeshDef) -> Self {
        match mesh_def.mesh_type {
            MeshTypeDef::Box {
                half_extents: [x, y, z],
            } => Self::Box(BoxMesh::new(-x, x, -y, y, -z, z)),
            _ => unimplemented!(),
        }
    }
}
