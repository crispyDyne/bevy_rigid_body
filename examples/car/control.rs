use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct CarControl {
    pub throttle: f32,
    pub steering: f32,
    pub brake: f32,
}
pub fn gamepad_system(
    gamepads: Res<Gamepads>,
    button_axes: Res<Axis<GamepadButton>>,
    axes: Res<Axis<GamepadAxis>>,
    mut control: ResMut<CarControl>,
) {
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
}
