use bevy::prelude::*;

use crate::joint::Joint;

pub fn get_model_state(joint_query: &Query<&mut Joint>) -> StateVector {
    let mut states = Vec::new();
    for joint in joint_query.iter() {
        states.push(joint.get_state());
    }
    StateVector { states }
}

pub fn get_model_dstate(joint_query: &Query<&mut Joint>) -> StateVector {
    let mut states = Vec::new();
    for joint in joint_query.iter() {
        states.push(joint.get_dstate());
    }
    StateVector { states }
}

pub fn set_model_state(joint_query: &mut Query<&mut Joint>, states: &StateVector) {
    for (joint_ind, mut joint) in joint_query.iter_mut().enumerate() {
        joint.set_state(states.states[joint_ind]);
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct State {
    pub q: f32,
    pub qd: f32,
}

impl std::ops::Mul<f32> for State {
    type Output = State;

    fn mul(self, rhs: f32) -> Self::Output {
        State {
            q: self.q * rhs,
            qd: self.qd * rhs,
        }
    }
}

impl std::ops::Mul<State> for f32 {
    type Output = State;

    fn mul(self, rhs: State) -> Self::Output {
        State {
            q: self * rhs.q,
            qd: self * rhs.qd,
        }
    }
}

impl std::ops::Add for State {
    type Output = State;

    fn add(self, rhs: State) -> Self::Output {
        State {
            q: self.q + rhs.q,
            qd: self.qd + rhs.qd,
        }
    }
}

#[derive(Default, Debug, Clone)]
pub struct StateVector {
    pub states: Vec<State>,
}

impl std::ops::Add for StateVector {
    type Output = StateVector;

    fn add(self, rhs: StateVector) -> Self::Output {
        StateVector {
            states: self
                .states
                .iter()
                .zip(rhs.states.iter())
                .map(|(a, b)| *a + *b)
                .collect(),
        }
    }
}

impl std::ops::Mul<f32> for StateVector {
    type Output = StateVector;

    fn mul(self, rhs: f32) -> Self::Output {
        StateVector {
            states: self.states.iter().map(|a| *a * rhs).collect(),
        }
    }
}
