mod camera;
mod physics;
mod setup;

use crate::camera::OrbitCameraPlugin;

use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugLinesPlugin;
use physics::{
    rigid_body::{RigidBody, RigidBodyBundle},
    PhysicsParameters, PhysicsPlugin, Radius,
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
            stiffness: 0.5,
            frequency: 60.0,
            time_scale: 1.0,
        })
        .add_system(bevy::window::close_on_esc)
        .add_startup_system(setup::light)
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
    let mut height = 0.0;
    for _ in 0..50 {
        let radius = random::<f32>() / 2.0 + 0.5;
        height += 2.5 * radius;
        commands.spawn((
            Radius(radius),
            RigidBodyBundle::from(RigidBody {
                mass: (random::<f32>() + 0.1) * 5.0,
                ..default()
            }),
            PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Icosphere {
                    radius: radius,
                    subdivisions: 32,
                })),
                material: materials.add(Color::hsl(random::<f32>() * 360.0, 1.0, 0.8).into()),
                transform: Transform::from_translation(Vec3::new(
                    random::<f32>() * 0.1,
                    height,
                    random::<f32>() * 0.1,
                )),
                ..default()
            },
        ));
    }
}
