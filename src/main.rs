mod camera;
mod physics;
mod setup;

use crate::camera::OrbitCameraPlugin;

use bevy::prelude::*;
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
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(PhysicsPlugin { substeps: 10 })
        .insert_resource(PhysicsParameters {
            debug: true,
            gravity: 10.0,
            stiffness: 1.0,
            frequency: 60.0,
            time_scale: 1.0,
        })
        .add_systems(Startup, setup::camera)
        .add_systems(Update, setup::axes)
        .add_systems(Startup, init);

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
        Linear::default().velocity(Vec3::X),
        Angular::default(),
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
                translation: Vec3::new(0.0, radius + 0.5 * length, 0.0),
                rotation: Quat::IDENTITY,
                scale: Vec3::ONE,
            },
            ..default()
        },
    ));
}
