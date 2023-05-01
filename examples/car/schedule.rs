use crate::physics::{
    brake_wheel_system, driven_wheel_system, steering_system, suspension_system,
    tire_contact_system,
};
use bevy::prelude::*;

use bevy_integrator::integrator::PhysicsScheduleExt;
use bevy_rigid_body::joint::Joint;
use bevy_rigid_body::structure::{apply_external_forces, loop_1, loop_23};

pub fn create_schedule() -> Schedule {
    let mut physics_schedule = Schedule::new();
    physics_schedule.add_physics_systems::<Joint, _, _, _>(
        (steering_system, loop_1).chain(),
        (
            suspension_system,
            tire_contact_system,
            driven_wheel_system,
            brake_wheel_system,
        ),
        (apply_external_forces, loop_23).chain(),
    );

    physics_schedule
}
