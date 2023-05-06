use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct CarControl {
    pub throttle: f32,
    pub steering: f32,
    pub brake: f32,
}

pub fn user_control_system(
    keyboard_input: Res<Input<KeyCode>>,
    gamepads: Res<Gamepads>,
    button_axes: Res<Axis<GamepadButton>>,
    axes: Res<Axis<GamepadAxis>>,
    mut control: ResMut<CarControl>,
) {
    // gamepad controls
    for gamepad in gamepads.iter() {
        control.throttle = button_axes
            .get(GamepadButton::new(
                gamepad,
                GamepadButtonType::RightTrigger2,
            ))
            .unwrap();

        control.brake = button_axes
            .get(GamepadButton::new(gamepad, GamepadButtonType::LeftTrigger2))
            .unwrap();

        control.steering = -axes
            .get(GamepadAxis::new(gamepad, GamepadAxisType::LeftStickX))
            .unwrap();
    }

    // keyboard controls - these are rate controlled to make them feel more natural
    // when a key is pressed, the control value is increased at a constant rate
    // when a key is released, the control value is decreased at a constant rate
    // the control value is clamped between 0 and 1 for throttle and brake, and
    // between -1 and 1 for steering
    let response_time = 0.25;
    let time_constant = 1. / (response_time * 60.);
    if keyboard_input.pressed(KeyCode::W) {
        control.throttle += time_constant;
        control.throttle = control.throttle.min(1.0);
    } else {
        control.throttle -= time_constant;
        control.throttle = control.throttle.max(0.0);
    }

    if keyboard_input.pressed(KeyCode::S) {
        control.brake += time_constant;
        control.brake = control.brake.min(1.0);
    } else {
        control.brake -= time_constant;
        control.brake = control.brake.max(0.0);
    }

    let mut steer_active = false;
    if keyboard_input.pressed(KeyCode::A) {
        control.steering += time_constant;
        control.steering = control.steering.min(1.0);
        steer_active = true;
    }

    if keyboard_input.pressed(KeyCode::D) {
        control.steering -= time_constant;
        control.steering = control.steering.max(-1.0);
        steer_active = true;
    }

    if !steer_active {
        if control.steering.abs() < time_constant {
            control.steering = 0.0;
        } else if control.steering > 0.0 {
            control.steering -= time_constant;
        } else {
            control.steering += time_constant;
        }
    }
}
