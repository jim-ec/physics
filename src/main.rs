mod camera;
mod constraint;
mod convert;
mod rigid_body;
mod setup;
mod util;

use crate::camera::OrbitCameraPlugin;

use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugLinesPlugin;
use parry3d::{
    math::{Isometry, Translation},
    na::Unit,
    query::contact,
    shape::{Ball, HalfSpace},
    simba::scalar::{SubsetOf, SupersetOf},
};
use rigid_body::RigidBody;
use util::Vector;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(0.9, 0.9, 0.9)))
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::with_depth_test(false))
        .add_plugin(OrbitCameraPlugin)
        .add_system(bevy::window::close_on_esc)
        .add_startup_system(setup::light)
        .add_startup_system(setup::camera)
        .add_startup_system(setup::axes)
        .add_startup_system(setup)
        .add_system(ground_contacts)
        .run()
}

const RADIUS: f32 = 1.0;
const GRAVITY: f32 = -10.0;
const SUBSTEPS: usize = 10;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let spawn_position = Vector::new(0.0, 4.0, 0.0);

    let body = RigidBody {
        force: Vector::new(0.0, GRAVITY, 0.0),
        inverse_mass: 1.0 / 1.0,
        velocity: Vector::zeros(),
        translation: spawn_position,
    };

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Icosphere {
                radius: RADIUS,
                subdivisions: 32,
            })),
            material: materials.add(Color::WHITE.into()),
            transform: Transform::from_translation(convert::vec(spawn_position)),
            ..default()
        },
        body,
    ));
}

fn ground_contacts(
    mut query: Query<(
        &mut RigidBody,
        &mut Transform,
        &mut Handle<StandardMaterial>,
    )>,
    time: Res<Time>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if time.delta_seconds() <= 0.0 {
        return;
    }

    for (mut body, mut transform, material) in query.iter_mut() {
        for _ in 0..SUBSTEPS {
            let dt = time.delta_seconds() / SUBSTEPS as f32;

            let past_translation = body.translation;
            body.integrate(dt);
            let integrated_translation = body.translation;

            if let Some(contact) = contact::contact(
                &Isometry::identity(),
                &HalfSpace::new(Unit::new_normalize(Vector::new(0.0, 1.0, 0.0))),
                &Isometry::from_subset(&Translation::new(
                    integrated_translation.x,
                    integrated_translation.y,
                    integrated_translation.z,
                )),
                &Ball { radius: RADIUS },
                0.0,
            )
            .unwrap()
            {
                const FACTOR: f32 = 1.0;
                body.apply_impulse(FACTOR * contact.dist * -contact.normal1.to_superset());

                materials.get_mut(&material).unwrap().base_color = Color::RED;
            } else {
                materials.get_mut(&material).unwrap().base_color = Color::WHITE;
            }

            body.derive(past_translation, dt);
        }

        transform.translation = convert::vec(body.translation);
    }
}
