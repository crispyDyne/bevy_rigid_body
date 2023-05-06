use bevy::prelude::*;

use crate::{
    joint::Joint,
    serialize::{BrakeWheelDef, DrivenWheelDef, SteeringDef, SuspensionDef, TireContactDef},
    sva::{Force, Vector},
};

use super::control::CarControl;

#[derive(Component)]
pub struct Suspension {
    stiffness: f32,
    damping: f32,
}

impl Suspension {
    pub fn new(stiffness: f32, damping: f32) -> Self {
        Self { stiffness, damping }
    }

    pub fn from_def(suspension_def: &SuspensionDef) -> Self {
        Self::new(suspension_def.stiffness, suspension_def.damping)
    }
}

pub fn suspension_system(mut joints: Query<(&mut Joint, &Suspension)>) {
    for (mut joint, suspension) in joints.iter_mut() {
        joint.tau -= suspension.stiffness * joint.q + suspension.damping * joint.qd;
    }
}

#[derive(Component)]
pub struct TireContact {
    radius: f32,
    stiffness: f32,
    damping: f32,
    longitudinal_stiffness: f32,
    lateral_stiffness: f32,
}

impl TireContact {
    pub fn new(
        radius: f32,
        stiffness: f32,
        damping: f32,
        longitudinal_stiffness: f32,
        lateral_stiffness: f32,
    ) -> Self {
        Self {
            radius,
            stiffness,
            damping,
            longitudinal_stiffness,
            lateral_stiffness,
        }
    }

    pub fn from_def(tire_contact_def: &TireContactDef) -> Self {
        Self::new(
            tire_contact_def.radius,
            tire_contact_def.stiffness,
            tire_contact_def.damping,
            tire_contact_def.longitudinal_stiffness,
            tire_contact_def.lateral_stiffness,
        )
    }
}

// a very simple tire model. Not very realistic, but it works well enough for this demo.
// it's also messy, but I/we can clean it up later
pub fn tire_contact_system(mut joints: Query<(&mut Joint, &TireContact)>) {
    for (mut joint, contact) in joints.iter_mut() {
        // x0 is the inverse of the joint transform (used several times later)
        let x0 = joint.x.inverse();

        // we care about two reference frames: the contact and the tire
        // the y axis of the tire reference frame is the axis of rotation of the tire
        // the y axis of the contact reference frame is the axis of rotation of the tire projected onto the ground (no z component)
        // the z axis of the contact reference frame is the up vector in absolute coordinates
        // the x axis of the contact reference frame is the cross product of the y and z axes
        // the x axis of the tire reference frame is the same as the x axis of the contact reference frame
        // the z axis of the tire reference frame is the cross product of the x and y axes

        // each of the reference frames can be written in absolute coordinates or local coordinates
        // wheel center in absolute coordinates
        let tire_point_abs = x0.transform_point(Vector::new(0., 0., 0.)); // wheel center in absolute coordinates

        // y axis of tire reference frame in local coordinates
        let tire_lat_local = Vector::new(0., 1., 0.); // axis of tire rotation

        // y axis of contact reference frame in absolute coordinates
        let tire_lat_abs = x0 * tire_lat_local;
        let mut contact_lat_abs = tire_lat_abs;
        contact_lat_abs.z = 0.; // axis of tire rotation projected onto the ground (no z component)
        let contact_lat_abs = contact_lat_abs.normalize();

        // z axis of contact reference frame in absolute coordinates
        let contact_up_abs = Vector::new(0., 0., 1.); // up vector in absolute coordinates

        // x axis of contact reference frame in absolute coordinates
        let contact_forward_abs = contact_lat_abs.cross(&contact_up_abs).normalize();

        // up vector of tire reference frame in absolute coordinates
        let tire_up_abs = contact_forward_abs.cross(&tire_lat_abs).normalize();

        // contact point in absolute coordinates
        let height = tire_point_abs.z / tire_up_abs.z; // height of the wheel center along the tire up vector
        let contact_point_abs = tire_point_abs - height * tire_up_abs; // subtract the height from the wheel center to get the contact point
        let deflection = contact.radius - height; // deflection of the tire

        if deflection > 0. {
            // vertical forces
            let spring_force = contact.stiffness * deflection;

            let v0 = x0 * joint.v;
            let vel_abs = v0.velocity_point(contact_point_abs);
            let damping_force = -contact.damping * tire_up_abs.dot(&vel_abs.vel);
            let vertical_force = spring_force + damping_force;

            // ground plane forces
            let forward_vel = vel_abs.vel.dot(&contact_forward_abs); // component of velocity in the forward direction
            let lat_vel = vel_abs.vel.dot(&contact_lat_abs); // component of velocity in the lateral direction
            let mut forward_force = -forward_vel * contact.longitudinal_stiffness * vertical_force;
            let mut lat_force = -lat_vel * contact.lateral_stiffness * vertical_force;

            forward_force = forward_force.max(-vertical_force).min(vertical_force);
            lat_force = lat_force.max(-vertical_force).min(vertical_force);

            let f_abs = Force::force_point(
                forward_force * contact_forward_abs
                    + lat_force * contact_lat_abs
                    + Vector::new(0., 0., vertical_force),
                contact_point_abs,
            );
            joint.f_ext += f_abs;
        }
    }
}

#[derive(Component)]
pub struct Steering {
    pub max_angle: f32,
}

impl Steering {
    pub fn new(max_angle: f32) -> Self {
        Self { max_angle }
    }

    pub fn from_def(steering_def: &SteeringDef) -> Self {
        Self::new(steering_def.max_angle)
    }
}

pub fn steering_system(mut joints: Query<(&mut Joint, &Steering)>, control: Res<CarControl>) {
    for (mut joint, steering) in joints.iter_mut() {
        joint.q = control.steering * steering.max_angle;
    }
}

#[derive(Component)]
pub struct DrivenWheel {
    pub max_torque: f32,
}

impl DrivenWheel {
    pub fn new(max_torque: f32) -> Self {
        Self { max_torque }
    }

    pub fn from_def(driven_wheel_def: &DrivenWheelDef) -> Self {
        Self::new(driven_wheel_def.max_torque)
    }
}

pub fn driven_wheel_system(
    mut joints: Query<(&mut Joint, &DrivenWheel)>,
    control: Res<CarControl>,
) {
    for (mut joint, driven_wheel) in joints.iter_mut() {
        joint.tau += control.throttle * driven_wheel.max_torque;
    }
}

#[derive(Component)]
pub struct BrakeWheel {
    pub max_torque: f32,
}

impl BrakeWheel {
    pub fn new(max_torque: f32) -> Self {
        Self { max_torque }
    }

    pub fn from_def(brake_wheel_def: &BrakeWheelDef) -> Self {
        Self::new(brake_wheel_def.max_torque)
    }
}

pub fn brake_wheel_system(mut joints: Query<(&mut Joint, &BrakeWheel)>, control: Res<CarControl>) {
    for (mut joint, brake_wheel) in joints.iter_mut() {
        joint.tau += -control.brake * brake_wheel.max_torque * joint.qd.signum();
    }
}
