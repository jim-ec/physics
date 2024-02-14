use std::f32::consts::TAU;

use bevy::input::mouse::MouseWheel;
use bevy::math::vec2;
use bevy::prelude::*;
use lerp::Lerp;

pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, camera);
    }
}

#[derive(Component)]
pub struct OrbitCamera {
    /// Angle between the up-vector and the camera position.
    pub inclination: f32,

    /// Angle between the x-axis and the camera position projected onto the xz-plane.
    pub azimuth: f32,

    /// Distance between the camera position from the origin.
    pub radius: f32,

    target_inclination: f32,
    target_azimuth: f32,
}

impl OrbitCamera {
    pub fn new(radius: f32, azimuth: f32, inclination: f32) -> Self {
        Self {
            inclination,
            azimuth,
            radius,
            target_inclination: inclination,
            target_azimuth: azimuth,
        }
    }
}

fn camera(
    mut wheel: EventReader<MouseWheel>,
    keys: Res<Input<KeyCode>>,
    mut query: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    let scroll = wheel
        .read()
        .fold(Vec2::ZERO, |acc, ev| acc + vec2(ev.x, ev.y));

    for (mut orbit_camera, mut transform) in query.iter_mut() {
        const SENSITIVITY: f32 = 0.008;
        const ORBIT_ADAPTATION: f32 = 0.4;

        if keys.pressed(KeyCode::SuperLeft) {
            orbit_camera.radius *= 1.0 - 0.001 * scroll.y;
        } else {
            orbit_camera.target_azimuth += SENSITIVITY * scroll.x;
            orbit_camera.target_inclination -= SENSITIVITY * scroll.y;
        }

        orbit_camera.azimuth = orbit_camera
            .azimuth
            .lerp(orbit_camera.target_azimuth, ORBIT_ADAPTATION);

        orbit_camera.target_inclination =
            orbit_camera.target_inclination.clamp(0.1, TAU / 2.0 - 0.1);
        orbit_camera.inclination = orbit_camera
            .inclination
            .lerp(orbit_camera.target_inclination, ORBIT_ADAPTATION);

        *transform = Transform::from_xyz(
            orbit_camera.radius * orbit_camera.inclination.sin() * orbit_camera.azimuth.cos(),
            orbit_camera.radius * orbit_camera.inclination.cos(),
            orbit_camera.radius * orbit_camera.inclination.sin() * orbit_camera.azimuth.sin(),
        )
        .looking_at(Vec3::ZERO, Vec3::Y);
    }
}
