use bevy::prelude::*;

mod build;
mod build_from_json;
mod camera_az_el;
mod control;
mod create_car_json;
mod environment;
mod physics;
mod plugin;
mod schedule;

use plugin::{CarPlugin, Mode};

// set a larger timestep if the animation lags
const FIXED_TIMESTEP: f32 = 0.002; // 0.002 -> 500 fps

// Main function
fn main() {
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
        .add_plugin(CarPlugin { mode: Mode::None })
        .run(); // update the bevy joint positions
}
