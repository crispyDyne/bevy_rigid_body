use crate::physics::{
    brake_wheel_system, driven_wheel_system, steering_system, suspension_system,
    tire_contact_system,
};
use bevy::prelude::*;

use bevy_integrator::{
    integrator::{PhysicsScheduleExt, PhysicsState, Stateful},
    recorder::RecordedData,
};
use bevy_rigid_body::{
    joint::Joint,
    structure::{apply_external_forces, loop_1, loop_23},
};

// simulation
pub fn create_physics_schedule() -> Schedule {
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

// replay
pub fn set_replay_data(
    recorded_data: Res<RecordedData>,
    time: Res<Time>,
    mut query: Query<(&mut Joint, Entity)>,
    mut physics_state: ResMut<PhysicsState<Joint>>,
) {
    let time_data = recorded_data.data.get("time").unwrap();
    // binary search to find the index of the current time
    let index = bin_search(time.elapsed_seconds(), time_data);
    for (mut joint, joint_entity) in query.iter_mut() {
        // get the state and dstate data for this joint
        let joint_name = joint.name.clone();
        let state_name = format!("{}_state", joint_name);
        let dstate_name = format!("{}_dstate", joint_name);
        let state_data = recorded_data.data.get(state_name.as_str()).unwrap();
        let dstate_data = recorded_data.data.get(dstate_name.as_str()).unwrap();

        // set the state and dstate of the joint (should eventually use interpolation)
        joint.q = state_data[index];
        joint.qd = dstate_data[index];

        // update the physics state
        physics_state.states.insert(joint_entity, joint.get_state());
    }
    // let t = recorded_data.data.get("time").unwrap()[index];
    // let px = recorded_data.data.get("chassis_px_state").unwrap()[index];
    // println!("{} {}", t, px)
}

fn bin_search(x: f32, data: &Vec<f32>) -> usize {
    let mut left = 0;
    let mut right = data.len() - 1;
    while left < right {
        let middle = (left + right) / 2;
        if data[middle] < x {
            left = middle + 1;
        } else {
            right = middle;
        }
    }
    left
}
