// allow unused code for now
#![allow(dead_code)]

use crate::joint::{Base, Joint};
use crate::state::{get_model_dstate, set_model_state, StateVector};
use bevy::prelude::*;

use crate::algorithms::{apply_external_update, loop_1_update, loop_2_update, loop_3_update};

pub fn all_loops(
    states: &StateVector,
    base_query: &Query<Entity, With<Base>>,
    joint_children_query: &Query<&Children, With<Joint>>,
    mut joint_query: &mut Query<&mut Joint>,
) -> StateVector {
    set_model_state(&mut joint_query, &states);

    base_loop(
        &base_query,
        &joint_children_query,
        &mut joint_query,
        Some(loop_1_update),
        None,
    );

    base_loop(
        &base_query,
        &joint_children_query,
        &mut joint_query,
        None,
        Some(loop_2_update),
    );

    base_loop(
        &base_query,
        &joint_children_query,
        &mut joint_query,
        Some(loop_3_update),
        None,
    );

    let dstate = get_model_dstate(&joint_query);
    dstate
}

pub fn loop_1(
    base_query: Query<Entity, With<Base>>,
    joint_children_query: Query<&Children, With<Joint>>,
    mut joint_query: Query<&mut Joint>,
) {
    base_loop(
        &base_query,
        &joint_children_query,
        &mut joint_query,
        Some(loop_1_update),
        None,
    );
}

pub fn apply_external_forces(
    base_query: Query<Entity, With<Base>>,
    joint_children_query: Query<&Children, With<Joint>>,
    mut joint_query: Query<&mut Joint>,
) {
    base_loop(
        &base_query,
        &joint_children_query,
        &mut joint_query,
        Some(apply_external_update),
        None,
    );
}

pub fn loop_23(
    base_query: Query<Entity, With<Base>>,
    joint_children_query: Query<&Children, With<Joint>>,
    mut joint_query: Query<&mut Joint>,
) {
    base_loop(
        &base_query,
        &joint_children_query,
        &mut joint_query,
        None,
        Some(loop_2_update),
    );

    base_loop(
        &base_query,
        &joint_children_query,
        &mut joint_query,
        Some(loop_3_update),
        None,
    );
}

pub fn base_loop(
    base_query: &Query<Entity, With<Base>>,
    joint_children_query: &Query<&Children, With<Joint>>,
    mut joint_query: &mut Query<&mut Joint>,
    fn_out: Option<fn(&mut Joint, &Joint)>,
    fn_in: Option<fn(&mut Joint, Option<&mut Joint>)>,
) {
    for base_entity in base_query.iter() {
        if let Ok(children) = joint_children_query.get(base_entity) {
            for child_entity in children.iter() {
                recursive_loop(
                    base_entity,
                    &child_entity,
                    &joint_children_query,
                    &mut joint_query,
                    fn_out,
                    fn_in,
                );
            }
        }
    }
}

pub fn recursive_loop(
    parent_entity: Entity,
    joint_entity: &Entity,
    joint_children_query: &Query<&Children, With<Joint>>,
    joint_query: &mut Query<&mut Joint>,
    fn_out: Option<fn(&mut Joint, &Joint)>,
    fn_in: Option<fn(&mut Joint, Option<&mut Joint>)>,
) {
    match fn_out {
        Some(f) => {
            // get parent and joint
            if let Ok([parent, mut joint]) =
                joint_query.get_many_mut([parent_entity, *joint_entity])
            {
                // call fn_out - outward pass, ordered from parent to child
                f(&mut joint, &parent);
            }
        }
        None => (),
    }

    match joint_children_query.get(*joint_entity) {
        Ok(children) => {
            // joint has children. loop through them.
            for child_entity in children.iter() {
                recursive_loop(
                    *joint_entity,
                    child_entity,
                    joint_children_query,
                    joint_query,
                    fn_out,
                    fn_in,
                );
            }
        }
        Err(_e) => {
            // joint has no children. This is fine. Do nothing.
        }
    }

    // get parent and joint
    match fn_in {
        Some(f) => {
            if let Ok([mut parent, mut joint]) =
                joint_query.get_many_mut([parent_entity, *joint_entity])
            {
                // call fn_in - inward pass, ordered from child to parent
                f(&mut joint, Some(&mut parent));
            }
        }
        None => (),
    }
}
