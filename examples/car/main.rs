use bevy::prelude::*;

use bevy_rigid_body::car::plugin::{CarPlugin, Mode};

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
        .add_plugin(CarPlugin {
            mode: Mode::None,
            time_step: 0.002, // 0.002 -> 500 fps
            camera: true,
            environment: true,
        })
        .run(); // update the bevy joint positions
}
