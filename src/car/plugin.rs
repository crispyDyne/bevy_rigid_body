use crate::{
    joint::{bevy_joint_positions, Joint},
    structure::loop_1,
};
use bevy::prelude::*;
use bevy_integrator::{
    integrator::{initialize_state, integrator_schedule, PhysicsSchedule, Solver},
    recorder::{create_recorder, initialize_recorder, load_recorded_data, recorder_system},
};

use super::{
    build, build_from_json,
    camera_az_el::{self, camera_builder},
    control::{self, CarControl},
    create_car_json::car_json,
    environment::build_environment,
    schedule::{create_physics_schedule, set_replay_data},
};

#[derive(Default)]
pub enum Mode {
    Record,
    Playback,
    #[default]
    None,
}

pub struct CarPlugin {
    pub mode: Mode,
    pub time_step: f32,
    pub camera: bool,
}

impl CarPlugin {
    pub fn setup_physics_simulation(&self, app: &mut App) {
        // run the physics simulation with user control
        let schedule = create_physics_schedule();
        app.add_schedule(PhysicsSchedule, schedule) // add the physics schedule
            .insert_resource(Solver::RK4) // set the solver to use
            .insert_resource(FixedTime::new_from_secs(self.time_step)) // set the fixed timestep
            .add_system(integrator_schedule::<Joint>.in_schedule(CoreSchedule::FixedUpdate)) // run the physics schedule in the fixed timestep loop
            .add_system(control::user_control_system) // control the car with a gamepad
            .init_resource::<CarControl>();
    }
}

impl Plugin for CarPlugin {
    fn build(&self, app: &mut App) {
        match self.mode {
            Mode::Record => {
                app.add_startup_system(create_recorder)
                    .add_startup_system(
                        initialize_recorder::<Joint>.in_base_set(StartupSet::PostStartup),
                    )
                    .add_system(recorder_system::<Joint>);
                self.setup_physics_simulation(app)
            }
            Mode::Playback => {
                app.add_startup_system(load_recorded_data).add_systems(
                    (
                        set_replay_data, // sets the joint position data
                        loop_1,          // calculates joint transforms
                    )
                        .chain(),
                );
            }
            Mode::None => self.setup_physics_simulation(app),
        }

        // camera setup
        if self.camera {
            app.add_startup_system(camera_builder(
                Vec3 {
                    x: 0.,
                    y: 10.,
                    z: 1.,
                },
                0.0_f32.to_radians(),
                10.0_f32.to_radians(),
                20.,
                camera_az_el::UpDirection::Z,
            ))
            .add_system(camera_az_el::az_el_camera); // setup the cameraw
        }

        // car setup
        app.add_startup_system(setup_system) // setup the car model and environment
            .add_startup_systems(
                (initialize_state::<Joint>,)
                    .chain()
                    .in_base_set(StartupSet::PostStartup),
            )
            .add_system(bevy_joint_positions);
    }
}

pub fn setup_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if true {
        car_json();
        build_from_json::build_model(&mut commands, &mut meshes, &mut materials);
    } else {
        build::build_model(&mut commands, &mut meshes, &mut materials);
    }
    build_environment(&mut commands, &mut meshes, &mut materials);
}
