use crate::{camera::OrbitCamera, CAMERA_DISTANCE};
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugLines;

pub fn axes(mut lines: ResMut<DebugLines>) {
    let red = Color::rgb(1.0, 0.1, 0.1);
    lines.line_colored(Vec3::ZERO, Vec3::X, f32::INFINITY, red);
    lines.line_colored(Vec3::X, Vec3::new(0.8, 0.0, 0.2), f32::INFINITY, red);
    lines.line_colored(Vec3::X, Vec3::new(0.8, 0.0, -0.2), f32::INFINITY, red);

    let green = Color::rgb(0.2, 1.0, 0.2);
    lines.line_colored(Vec3::ZERO, Vec3::Y, f32::INFINITY, green);
    lines.line_colored(Vec3::Y, Vec3::new(0.2, 0.8, 0.0), f32::INFINITY, green);
    lines.line_colored(Vec3::Y, Vec3::new(-0.2, 0.8, 0.0), f32::INFINITY, green);

    let blue = Color::rgb(0.3, 0.6, 1.0);
    lines.line_colored(Vec3::ZERO, Vec3::Z, f32::INFINITY, blue);
    lines.line_colored(Vec3::Z, Vec3::new(0.2, 0.0, 0.8), f32::INFINITY, blue);
    lines.line_colored(Vec3::Z, Vec3::new(-0.2, 0.0, 0.8), f32::INFINITY, blue);
}

pub fn camera(mut commands: Commands) {
    commands
        .spawn((
            Camera3dBundle {
                transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
                ..default()
            },
            OrbitCamera::new(
                CAMERA_DISTANCE,
                f32::to_radians(50.0),
                f32::to_radians(60.0),
            ),
            Visibility::VISIBLE,
            ComputedVisibility::default(),
        ))
        .with_children(|children| {
            children.spawn(DirectionalLightBundle::default());
        });
}
