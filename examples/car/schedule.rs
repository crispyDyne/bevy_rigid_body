use crate::physics::{
    driven_wheel_system, steering_system, suspension_system, tire_contact_system,
};
use bevy::ecs::schedule::ScheduleLabel;
use bevy::prelude::*;

use bevy_rigid_body::algorithms::integrate_joint_state;
use bevy_rigid_body::structure::{apply_external_forces, loop_1, loop_23};

// Define the physics schedule which will be run in the fixed timestep loop
#[derive(ScheduleLabel, Debug, Hash, PartialEq, Eq, Clone)]
pub struct PhysicsSchedule;

pub fn physics_schedule(world: &mut World) {
    world.run_schedule(PhysicsSchedule);
}

// Define physics system sets, which are used to group systems together, and define the order in which they are run
#[derive(Debug, Hash, PartialEq, Eq, Clone, SystemSet)]
enum PhysicsSet {
    Initialize,
    Physics,
    Finalize,
}

pub fn create_schedule() -> Schedule {
    // create the physics schedule
    let mut phys_schedule = Schedule::new();
    phys_schedule
        .configure_sets(
            (
                PhysicsSet::Initialize,
                PhysicsSet::Physics,
                PhysicsSet::Finalize,
            )
                .chain(), // This defines the ordering of the system sets
        )
        .add_systems(
            (
                steering_system, // set the steering angle first
                loop_1, // RBDA Solver: updates all joint positions and velocities based on their state and the state of their parents
            )
                .chain() // these systems should be run in order
                .in_set(PhysicsSet::Initialize),
        )
        .add_systems(
            (
                suspension_system,   // suspension forces
                tire_contact_system, //  tire forces
                driven_wheel_system, // torque applied to the driven wheels
            ) // no "chain" here, these systems can be run in any order
                .in_set(PhysicsSet::Physics),
        )
        .add_systems(
            (
                apply_external_forces, // RBDA Solver: applies the external forces to joints
                loop_23,               // RBDA Solver: calculates joint accelerations
                integrate_joint_state, // RBDA Solver: integrates the joint state
            )
                .chain() // these systems should be run in order
                .in_set(PhysicsSet::Finalize),
        );
    phys_schedule
}
