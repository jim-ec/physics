mod camera;
mod physics;
mod setup;

use std::f32::consts::TAU;

use crate::camera::OrbitCameraPlugin;

use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugLinesPlugin;
use physics::{
    collider::{Collider, Shape},
    motion::{Angular, Linear, Rigid},
    PhysicsParameters, PhysicsPlugin,
};
use rand::random;

pub const CAMERA_DISTANCE: f32 = 20.0;

fn main() {
    let mut app = App::new();

    app.insert_resource(ClearColor(Color::rgb(0.1, 0.1, 0.1)))
        .add_plugins(DefaultPlugins)
        .add_plugin(OrbitCameraPlugin)
        .add_plugin(PhysicsPlugin { substeps: 10 })
        .insert_resource(PhysicsParameters {
            debug: true,
            gravity: 10.0,
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
    commands.spawn((
        Collider {
            mass: f32::INFINITY,
            shape: Shape::Plane { normal: Vec3::Y },
        },
        Rigid::default(),
        TransformBundle::IDENTITY,
    ));

    let length = 1.0;
    let radius = 0.5;
    commands.spawn((
        Collider {
            mass: 1.0,
            shape: Shape::Capsule {
                length: 1.0,
                radius,
            },
        },
        Linear::default(),
        // Angular::default(),
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Capsule {
                radius,
                depth: length,
                latitudes: 32,
                longitudes: 64,
                ..default()
            })),
            material: materials.add(Color::hsl(random::<f32>() * 360.0, 1.0, 0.8).into()),
            transform: Transform {
                translation: Vec3::new(0.0, 2.0, 0.0),
                rotation: Quat::from_euler(EulerRot::default(), 0.0, TAU / 8.0, 0.0),
                // rotation: Quat::IDENTITY,
                scale: Vec3::ONE,
            },
            ..default()
        },
    ));
    commands.spawn((
        Collider {
            mass: 1.0,
            shape: Shape::Capsule {
                length: 1.0,
                radius,
            },
        },
        Rigid::default(),
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Capsule {
                radius,
                depth: length,
                latitudes: 32,
                longitudes: 64,
                ..default()
            })),
            material: materials.add(Color::hsl(random::<f32>() * 360.0, 1.0, 0.8).into()),
            transform: Transform {
                translation: Vec3::new(0.0, 4.0, 0.0),
                rotation: Quat::IDENTITY,
                scale: Vec3::ONE,
            },
            ..default()
        },
    ));
}
