use std::{collections::HashMap, fs::File, io::Read};

use super::physics::{BrakeWheel, DrivenWheel, Steering, Suspension, TireContact};
use crate::{
    joint::{Base, Joint},
    serialize::{JointTypeDef, MeshDef, MeshTypeDef, ModelDef, SystemTypeDef},
    sva::Motion,
};
use bevy::prelude::*;

pub fn build_model(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    // load the model from json
    let folder_name = "./data";
    let file_name = "car.json";
    let full_path = format!("{}/{}", folder_name, file_name);

    let mut file = File::open(full_path).unwrap();
    let mut json = String::new();
    file.read_to_string(&mut json).unwrap();

    // deserialize the model
    let model: ModelDef = serde_json::from_str(&json).unwrap();

    let joint_ids = build_joints(commands, meshes, materials, &model);
    build_systems(commands, &model, &joint_ids);
}

pub fn build_joints(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    model: &ModelDef,
) -> HashMap<String, Entity> {
    let mut joint_ids = HashMap::new();
    for joint_def in model.joints.iter() {
        let mut joint = Joint::from_joint_def(joint_def);

        // hard coded stuff for now
        // base acceleration (gravity)
        if joint_def.joint_type == JointTypeDef::Base {
            joint.a = Motion::new([0., 0., 9.81], [0., 0., 0.]);
        }
        // initial conditions
        if joint_def.name.clone() == "chassis_pz" {
            joint.q = 0.3 + 0.25;
        }

        // spawn joint
        let mut joint_e = commands.spawn((joint, SpatialBundle::default()));
        // insert base if this is the base joint
        if joint_def.joint_type == JointTypeDef::Base {
            joint_e.insert(Base);
        }

        // add id
        let joint_id = joint_e.id();
        joint_ids.insert(joint_def.name.clone(), joint_id);

        // set parent
        if let Some(parent) = &joint_def.parent {
            let parent_id = joint_ids.get(parent).expect("Parent not yet created. Joint names must be unique, and parents must be created before children.");
            joint_e.set_parent(*parent_id);
        }

        // create meshes
        for mesh_def in joint_def.meshes.iter() {
            build_mesh(commands, meshes, materials, joint_id, mesh_def);
        }
    }

    joint_ids
}

fn build_systems(commands: &mut Commands, model: &ModelDef, joint_ids: &HashMap<String, Entity>) {
    for system_def in model.systems.iter() {
        match &system_def.system_type {
            SystemTypeDef::Steering(steering_def) => {
                let steering_id = joint_ids.get(&steering_def.joint).unwrap();
                commands
                    .entity(*steering_id)
                    .insert(Steering::from_def(&steering_def));
            }
            SystemTypeDef::Drive(drive_def) => {
                let drive_id = joint_ids.get(&drive_def.joint).unwrap();
                let mut drive_e = commands.entity(*drive_id);
                drive_e.insert(DrivenWheel::from_def(&drive_def));
            }
            SystemTypeDef::Brake(brake_def) => {
                let brake_id = joint_ids.get(&brake_def.joint).unwrap();
                commands
                    .entity(*brake_id)
                    .insert(BrakeWheel::from_def(&brake_def));
            }
            SystemTypeDef::Suspension(suspension_def) => {
                let suspension_id = joint_ids.get(&suspension_def.joint).unwrap();
                commands
                    .entity(*suspension_id)
                    .insert(Suspension::from_def(&suspension_def));
            }
            SystemTypeDef::TireContact(tire_contact_def) => {
                let tire_contact_id = joint_ids.get(&tire_contact_def.joint).unwrap();
                commands
                    .entity(*tire_contact_id)
                    .insert(TireContact::from_def(&tire_contact_def));
            }
        }
    }
}

fn build_mesh(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    joint_id: Entity,
    mesh_def: &MeshDef,
) {
    let mesh = match &mesh_def.mesh_type {
        MeshTypeDef::Box { half_extents } => Mesh::from(shape::Box {
            max_x: half_extents[0],
            min_x: -half_extents[0],
            max_y: half_extents[1],
            min_y: -half_extents[1],
            max_z: half_extents[2],
            min_z: -half_extents[2],
        }),
        MeshTypeDef::Cylinder { height, radius } => Mesh::from(shape::Cylinder {
            height: *height,
            radius: *radius,
            ..Default::default()
        }),
        // MeshTypeDef::Sphere { radius } => todo!(),
        // MeshTypeDef::Mesh { filename } => todo!(),
    };
    let color = Color::rgb(mesh_def.color[0], mesh_def.color[1], mesh_def.color[2]);

    let transform = Transform {
        translation: Vec3::new(
            mesh_def.transform.position[0],
            mesh_def.transform.position[1],
            mesh_def.transform.position[2],
        ),
        rotation: Quat::from_xyzw(
            mesh_def.transform.quaternion[1],
            mesh_def.transform.quaternion[2],
            mesh_def.transform.quaternion[3],
            mesh_def.transform.quaternion[0],
        ),
        ..Default::default()
    };

    commands
        .spawn(PbrBundle {
            mesh: meshes.add(mesh),
            material: materials.add(StandardMaterial {
                base_color: color,
                ..default()
            }),
            transform,
            ..default()
        })
        .set_parent(joint_id);
}
