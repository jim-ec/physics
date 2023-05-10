mod camera;
mod constraint;
mod convert;
mod rigid_body;
mod setup;
mod util;

use crate::camera::OrbitCameraPlugin;

use bevy::prelude::*;
use bevy_prototype_debug_lines::{DebugLines, DebugLinesPlugin};
use parry3d::{
    math::{Isometry, Translation},
    query::contact,
    shape::Ball,
    simba::scalar::{SubsetOf, SupersetOf},
};
use rigid_body::RigidBody;
use util::Vector;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(0.1, 0.1, 0.1)))
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::with_depth_test(false))
        .add_plugin(OrbitCameraPlugin)
        .add_system(bevy::window::close_on_esc)
        .add_startup_system(setup::light)
        .add_startup_system(setup::camera)
        .add_startup_system(setup::axes)
        .add_startup_system(setup)
        .run()
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut lines: ResMut<DebugLines>,
) {
    let mut iso1: Isometry<f32> = Isometry::from_subset(&Translation::new(0.0, 0.0, 0.0));
    let mut iso2: Isometry<f32> = Isometry::from_subset(&Translation::new(1.5, 0.5, 0.0));
    let ball1 = Ball { radius: 1.0 };
    let ball2 = Ball { radius: 0.8 };

    let contact = contact::contact(&iso1, &ball1, &iso2, &ball2, 0.0)
        .unwrap()
        .unwrap();
    lines.line_colored(
        convert::point(contact.point1),
        convert::point(contact.point2),
        f32::MAX,
        Color::rgb(1.0, 1.0, 0.1),
    );

    let mut body1 = RigidBody {
        force: Vector::zeros(),
        inverse_mass: 1.0 / 10.0,
        velocity: Vector::zeros(),
        translation: iso1.translation.vector,
    };
    let mut body2 = RigidBody {
        force: Vector::zeros(),
        inverse_mass: 1.0 / 10.0,
        velocity: Vector::zeros(),
        translation: iso2.translation.vector,
    };

    const FACTOR: f32 = 1.0;
    body1.apply_impulse(FACTOR * -contact.normal1.to_superset());
    body2.apply_impulse(FACTOR * contact.normal1.to_superset());

    iso1.translation.vector = body1.translation;
    iso2.translation.vector = body2.translation;

    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Icosphere {
            radius: ball1.radius,
            subdivisions: 32,
        })),
        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        transform: convert::iso(iso1),
        ..default()
    });

    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Icosphere {
            radius: ball2.radius,
            subdivisions: 32,
        })),
        material: materials.add(Color::rgb(0.6, 0.7, 0.8).into()),
        transform: convert::iso(iso2),
        ..default()
    });
}
