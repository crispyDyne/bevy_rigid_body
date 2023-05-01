use bevy::prelude::*;

mod build;
mod camera_az_el;
mod control;
mod enviornment;
pub mod physics;
mod schedule;

use bevy_integrator::integrator::{initialize_state, integrator_schedule, PhysicsSchedule, Solver};
use bevy_rigid_body::joint::{bevy_joint_positions, Joint};
use build::build_model;
use camera_az_el::camera_builder;

use control::CarControl;
use enviornment::build_environment;
use schedule::create_schedule;

// set a larger timestep if the animation lags
const FIXED_TIMESTEP: f32 = 0.002; // 0.002 -> 500 fps

// Main function
fn main() {
    // Create the physics schedule
    let schedule = create_schedule();
    // Create App
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: (1200., 900.).into(),
                title: "Car Demo".to_string(),
                resizable: true,
                ..default()
            }),
            ..default()
        }))
        .add_startup_system(camera_builder(
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
        .add_system(camera_az_el::az_el_camera) // setup the camera
        .add_startup_system(setup_system) // setup the car model and environment
        .insert_resource(FixedTime::new_from_secs(FIXED_TIMESTEP)) // set the fixed timestep
        .add_schedule(PhysicsSchedule, schedule) // add the physics schedule
        .insert_resource(Solver::RK4) // set the solver to use
        .add_startup_system(initialize_state::<Joint>.in_base_set(StartupSet::PostStartup)) // setup the car model and environment
        .add_system(integrator_schedule::<Joint>.in_schedule(CoreSchedule::FixedUpdate)) // run the physics schedule in the fixed timestep loop
        .add_system(bevy_joint_positions) // update the bevy joint positions
        .add_system(control::gamepad_system) // control the car with a gamepad
        .init_resource::<CarControl>()
        .run();
}

pub fn setup_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    build_model(&mut commands, &mut meshes, &mut materials);
    build_environment(&mut commands, &mut meshes, &mut materials);
}
