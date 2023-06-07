use std::fs::File;
use std::{f32::consts::PI, io::Write};

use crate::serialize::{
    BrakeWheelDef, DrivenWheelDef, InertiaDef, JointDef, JointTypeDef, MeshDef, MeshTypeDef,
    ModelDef, SteeringDef, SuspensionDef, SystemDef, SystemTypeDef, TireContactDef, TransformDef,
};

const ZERO_INERTIA: InertiaDef = InertiaDef {
    mass: 0.,
    center_of_mass: [0., 0., 0.],
    inertia: [0., 0., 0., 0., 0., 0.],
};

const ZERO_TRANSFORM: TransformDef = TransformDef {
    position: [0., 0., 0.],
    quaternion: [1., 0., 0., 0.],
};

pub fn car_json() {
    let corner_names = vec!["fl", "fr", "rl", "rr"];

    let mut joints = Vec::new();
    let mut systems = Vec::new();

    // create base joint
    let base_joint = JointDef {
        name: "base".to_string(),
        joint_type: JointTypeDef::Base,
        parent: None,
        transform: ZERO_TRANSFORM,
        inertia: ZERO_INERTIA,
        meshes: Vec::new(),
    };
    joints.push(base_joint);

    // create joints and systems
    let chassis_name = chassis_joints(&mut joints);
    suspension_joints(&mut joints, &mut systems, chassis_name, &corner_names);
    steering_joints(&mut joints, &mut systems, &corner_names);
    wheel_joints(&mut joints, &mut systems, &corner_names);

    // define the model
    let model = ModelDef {
        name: "car".to_string(),
        joints,
        systems,
    };

    // serialize the model to json
    let json = serde_json::to_string_pretty(&model).unwrap();
    let folder_name = "./data";
    let file_name = "car.json";
    let full_path = format!("{}/{}", folder_name, file_name);

    // create folder if it doesn't exist
    if !std::path::Path::new(folder_name).exists() {
        std::fs::create_dir(folder_name).unwrap();
    }

    // write the json to file
    let mut file = File::create(full_path).unwrap();
    file.write_all(json.as_bytes()).unwrap();
}

fn chassis_joints(joints: &mut Vec<JointDef>) -> String {
    // define chassis joints - 6 dof (Px, Py, Pz,Rx, Ry, Rz)
    let chassis_joint_def = vec![
        JointTypeDef::Px,
        JointTypeDef::Py,
        JointTypeDef::Pz,
        JointTypeDef::Rz,
        JointTypeDef::Ry,
        JointTypeDef::Rx,
    ];
    let chassis_dims = [3.0_f32, 1.25, 0.4]; // approximate dimensions of a car
    let chassis_mass = 1000.;
    let moi_xx = chassis_mass / 12. * (chassis_dims[1].powi(2) + chassis_dims[2].powi(2));
    let moi_yy = chassis_mass / 12. * (chassis_dims[0].powi(2) + chassis_dims[2].powi(2));
    let moi_zz = chassis_mass / 12. * (chassis_dims[0].powi(2) + chassis_dims[1].powi(2));
    let chassis_inertia = InertiaDef {
        mass: chassis_mass,
        center_of_mass: [0., 0., 0.],
        inertia: [moi_xx, moi_yy, moi_zz, 0., 0., 0.], // xx, yy, zz, yz, xz, xy
    };
    let mut parent_name = "base".to_string();
    for i in 0..6 {
        let (inertia, meshes) = if i == 5 {
            // last joint is the chassis with inertia and a mesh
            let meshes = vec![MeshDef {
                mesh_type: MeshTypeDef::Box {
                    half_extents: [
                        chassis_dims[0] / 2.,
                        chassis_dims[1] / 2.,
                        chassis_dims[2] / 2.,
                    ],
                },
                transform: ZERO_TRANSFORM,
                color: [0.5, 0.5, 0.5, 1.],
            }];
            (chassis_inertia.clone(), meshes)
        } else {
            // all other joints have no inertia or mesh
            (ZERO_INERTIA, vec![])
        };

        let name = format!(
            "chassis_{}",
            chassis_joint_def[i].to_string().to_lowercase()
        );
        let chassis = JointDef {
            name: name.clone(),
            joint_type: chassis_joint_def[i].clone(),
            parent: Some(parent_name.clone()),
            transform: ZERO_TRANSFORM,
            inertia: inertia,
            meshes,
        };
        joints.push(chassis);
        parent_name = name;
    }
    parent_name
}

fn suspension_joints(
    joints: &mut Vec<JointDef>,
    systems: &mut Vec<SystemDef>,
    chassis_name: String,
    corner_names: &Vec<&str>,
) {
    let susp_positions = vec![
        [1.25, 0.75, -0.3],   // fl
        [1.25, -0.75, -0.3],  // fr
        [-1.25, 0.75, -0.3],  // rl
        [-1.25, -0.75, -0.3], // rr
    ];
    let susp_mass = 10.;
    let susp_moi = 2.0 / 3. * susp_mass * 0.25_f32.powi(2);
    let suspension_inertia = InertiaDef {
        mass: susp_mass,
        center_of_mass: [0., 0., 0.],
        inertia: [susp_moi, susp_moi, susp_moi, 0.0, 0.0, 0.0],
    };
    let stiffness: f32 = 1000. * 9.81 / 4. / 0.1; // weight / 4 / spring travel
    let damping = 0.5 * 2. * (stiffness * (1000. / 4.)).sqrt(); // some fraction of critical damping
    for i in 0..4 {
        let name = format!("suspension_{}", corner_names[i]);
        let suspension = JointDef {
            name: name.clone(),
            joint_type: JointTypeDef::Pz,
            parent: Some(chassis_name.clone()),
            transform: TransformDef {
                position: susp_positions[i],
                quaternion: [1., 0., 0., 0.],
            },
            inertia: suspension_inertia.clone(),
            meshes: vec![MeshDef {
                mesh_type: MeshTypeDef::Box {
                    half_extents: [0.125, 0.125, 0.125],
                },
                transform: ZERO_TRANSFORM,
                color: [1.0, 0.0, 0.0, 1.0],
            }],
        };
        joints.push(suspension);
        systems.push(SystemDef {
            system_type: SystemTypeDef::Suspension(SuspensionDef {
                joint: name,
                stiffness,
                damping,
            }),
        });
    }
}

fn steering_joints(
    joints: &mut Vec<JointDef>,
    systems: &mut Vec<SystemDef>,
    corner_names: &Vec<&str>,
) {
    for i in 0..2 {
        let name = format!("steering_{}", corner_names[i]);
        let parent_name = format!("suspension_{}", corner_names[i]);
        let steering = JointDef {
            name: name.clone(),
            joint_type: JointTypeDef::Rz,
            parent: Some(parent_name),
            transform: TransformDef {
                position: [0., 0., 0.],
                quaternion: [1., 0., 0., 0.],
            },
            inertia: InertiaDef {
                mass: 0.0,
                center_of_mass: [0., 0., 0.],
                inertia: [0.0, 0.0, 0.0, 0., 0., 0.],
            },
            meshes: vec![],
        };
        joints.push(steering);
        systems.push(SystemDef {
            system_type: SystemTypeDef::Steering(SteeringDef {
                joint: name,
                max_angle: 30. * PI / 180.,
            }),
        });
    }
}

fn wheel_joints(
    joints: &mut Vec<JointDef>,
    systems: &mut Vec<SystemDef>,
    corner_names: &Vec<&str>,
) {
    let wheel_mass = 10.;
    let moi_xz = 1. / 12. * wheel_mass * (3. * 0.25_f32.powi(2));
    let moi_y = wheel_mass * 0.25_f32.powi(2);

    for i in 0..4 {
        let name = format!("wheel_{}", corner_names[i]);
        // front corner parent is steering, rear corner parent is suspension
        let parent_name = if i < 2 {
            format!("steering_{}", corner_names[i])
        } else {
            format!("suspension_{}", corner_names[i])
        };
        let wheel = JointDef {
            name: name.clone(),
            joint_type: JointTypeDef::Ry,
            parent: Some(parent_name),
            transform: TransformDef {
                position: [0., 0., 0.],
                quaternion: [1., 0., 0., 0.],
            },
            inertia: InertiaDef {
                mass: wheel_mass,
                center_of_mass: [0., 0., 0.],
                inertia: [moi_xz, moi_y, moi_xz, 0., 0., 0.],
            },
            meshes: vec![MeshDef {
                mesh_type: MeshTypeDef::Cylinder {
                    height: 0.2,
                    radius: 0.325,
                },
                transform: ZERO_TRANSFORM,
                color: [0.5, 0.5, 1.0, 1.],
            }],
        };
        joints.push(wheel);

        // drive system for rear wheels
        if i >= 2 {
            systems.push(SystemDef {
                system_type: SystemTypeDef::Drive(DrivenWheelDef {
                    joint: name.clone(),
                    max_torque: 400.,
                    max_speed: 50.,
                    max_power: 100.0e3,
                }),
            });
        }

        // brake system for all wheels
        systems.push(SystemDef {
            system_type: SystemTypeDef::Brake(BrakeWheelDef {
                joint: name.clone(),
                max_torque: if i < 2 { 800. } else { 400. },
            }),
        });

        // tire contact system for all wheels
        let tire_stiffness = 1000. * 9.81 / 4. / 0.005;
        let tire_damping = 0.25 * 2. * (1000.0_f32 / 4. * tire_stiffness).sqrt();
        systems.push(SystemDef {
            system_type: SystemTypeDef::TireContact(TireContactDef {
                joint: name.clone(),
                radius: 0.325,
                stiffness: tire_stiffness,
                damping: tire_damping,
                longitudinal_stiffness: 0.2,
                lateral_stiffness: 0.5,
            }),
        });
    }
}
