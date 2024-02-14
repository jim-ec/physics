use crate::{camera::OrbitCamera, CAMERA_DISTANCE};
use bevy::prelude::*;

pub fn axes(mut gizmos: Gizmos) {
    let red = Color::rgb(1.0, 0.1, 0.1);
    gizmos.line(Vec3::ZERO, Vec3::X, red);
    gizmos.line(Vec3::X, Vec3::new(0.8, 0.0, 0.2), red);
    gizmos.line(Vec3::X, Vec3::new(0.8, 0.0, -0.2), red);

    let green = Color::rgb(0.2, 1.0, 0.2);
    gizmos.line(Vec3::ZERO, Vec3::Y, green);
    gizmos.line(Vec3::Y, Vec3::new(0.2, 0.8, 0.0), green);
    gizmos.line(Vec3::Y, Vec3::new(-0.2, 0.8, 0.0), green);

    let blue = Color::rgb(0.3, 0.6, 1.0);
    gizmos.line(Vec3::ZERO, Vec3::Z, blue);
    gizmos.line(Vec3::Z, Vec3::new(0.2, 0.0, 0.8), blue);
    gizmos.line(Vec3::Z, Vec3::new(-0.2, 0.0, 0.8), blue);
}

#[allow(unused)]
pub fn grid(mut gizmos: Gizmos) {
    let n = 100;

    gizmos.line(
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(n as f32, 0.0, 0.0),
        Color::WHITE,
    );
    gizmos.line(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(-n as f32, 0.0, 0.0),
        Color::WHITE,
    );
    gizmos.line(
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(0.0, 0.0, n as f32),
        Color::WHITE,
    );
    gizmos.line(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, -n as f32),
        Color::WHITE,
    );

    for i in 1..n {
        let color = if i % 10 == 0 {
            Color::WHITE
        } else {
            Color::GRAY
        };
        gizmos.line(
            Vec3::new(n as f32, 0.0, i as f32),
            Vec3::new(-n as f32, 0.0, i as f32),
            color,
        );
        gizmos.line(
            Vec3::new(n as f32, 0.0, -i as f32),
            Vec3::new(-n as f32, 0.0, -i as f32),
            color,
        );
        gizmos.line(
            Vec3::new(i as f32, 0.0, n as f32),
            Vec3::new(i as f32, 0.0, -n as f32),
            color,
        );
        gizmos.line(
            Vec3::new(-i as f32, 0.0, n as f32),
            Vec3::new(-i as f32, 0.0, -n as f32),
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
            VisibilityBundle::default(),
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
