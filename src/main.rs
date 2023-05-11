mod camera;
mod physics;
mod setup;

use crate::camera::OrbitCameraPlugin;

use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugLinesPlugin;
use physics::{
    collider::Collider,
    rigid_body::{RigidBody, RigidBodyBundle},
    PhysicsParameters, PhysicsPlugin,
};
use rand::random;

pub const CAMERA_DISTANCE: f32 = 20.0;

fn main() {
    let mut app = App::new();

    app.insert_resource(ClearColor(Color::rgb(0.1, 0.1, 0.1)))
        .add_plugins(DefaultPlugins)
        .add_plugin(OrbitCameraPlugin)
        .add_plugin(PhysicsPlugin { debug: true })
        .insert_resource(PhysicsParameters {
            gravity: 10.0,
            substeps: 10,
            stiffness: 1.0,
            frequency: 60.0,
            time_scale: 1.0,
        })
        .add_system(bevy::window::close_on_esc)
        .add_startup_system(setup::camera)
        .add_startup_system(setup::axes)
        .add_startup_system(init);

    if !app.is_plugin_added::<DebugLinesPlugin>() {
        app.add_plugin(DebugLinesPlugin::with_depth_test(false));
    }

    app.run();
}

fn init(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let radius = 0.5;
    let length = 1.0;

    commands.spawn((
        Collider::Capsule { length, radius },
        RigidBodyBundle::from(RigidBody { ..default() }),
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Capsule {
                radius,
                depth: length,
                latitudes: 32,
                longitudes: 64,
                ..default()
            })),
            material: materials.add(Color::hsl(random::<f32>() * 360.0, 1.0, 0.8).into()),
            transform: Transform::from_translation(Vec3::new(0.0, 4.0, 0.0)),
            ..default()
        },
    ));
}
