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

#[allow(unused)]
pub fn grid(mut lines: ResMut<DebugLines>) {
    let n = 100;

    lines.line_colored(
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(n as f32, 0.0, 0.0),
        f32::INFINITY,
        Color::WHITE,
    );
    lines.line_colored(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(-n as f32, 0.0, 0.0),
        f32::INFINITY,
        Color::WHITE,
    );
    lines.line_colored(
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, n as f32),
        f32::INFINITY,
        Color::WHITE,
    );
    lines.line_colored(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, -n as f32),
        f32::INFINITY,
        Color::WHITE,
    );

    for i in 1..n {
        let color = if i % 10 == 0 {
            Color::WHITE
        } else {
            Color::GRAY
        };
        lines.line_colored(
            Vec3::new(n as f32, 0.0, i as f32),
            Vec3::new(-n as f32, 0.0, i as f32),
            f32::INFINITY,
            color,
        );
        lines.line_colored(
            Vec3::new(n as f32, 0.0, -i as f32),
            Vec3::new(-n as f32, 0.0, -i as f32),
            f32::INFINITY,
            color,
        );
        lines.line_colored(
            Vec3::new(i as f32, 0.0, n as f32),
            Vec3::new(i as f32, 0.0, -n as f32),
            f32::INFINITY,
            color,
        );
        lines.line_colored(
            Vec3::new(-i as f32, 0.0, n as f32),
            Vec3::new(-i as f32, 0.0, -n as f32),
            f32::INFINITY,
            color,
        );
    }
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
            Visibility::Visible,
            ComputedVisibility::default(),
        ))
        .with_children(|children| {
            children.spawn(DirectionalLightBundle {
                directional_light: DirectionalLight {
                    illuminance: 60000.0,
                    ..default()
                },
                ..default()
            });
        });
}
