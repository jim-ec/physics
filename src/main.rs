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
    na::Unit,
    query::contact,
    shape::{Ball, HalfSpace},
    simba::scalar::{SubsetOf, SupersetOf},
};
use rand::random;
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
        .add_system(debug_bodies)
        .add_system(ball_contacts)
        .run()
}

const GRAVITY: f32 = 50.0;
const SUBSTEPS: usize = 10;
const STIFFNESS: f32 = 0.5;
const FREQUENCY: f32 = 60.0;
const TIME_SCALE: f32 = 1.0;

#[derive(Component, Clone, Copy)]
struct Radius(f32);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut height = 1.0;
    for _ in 0..50 {
        let radius = 0.5;
        let location = Vector::new(random::<f32>() * 0.1, height, random::<f32>() * 0.1);
        commands.spawn((
            Radius(radius),
            RigidBody::default()
                .force(Vector::new(0.0, -GRAVITY, 0.0))
                .inverse_mass(1.0 / 1.0)
                .teleport(location),
            PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Icosphere {
                    radius: radius,
                    subdivisions: 32,
                })),
                material: materials.add(Color::hsl(random::<f32>() * 360.0, 1.0, 0.8).into()),
                transform: Transform::from_translation(convert::vec(location)),
                ..default()
            },
        ));

        height += 1.5;
    }
}

fn ball_contacts(mut query: Query<(&mut RigidBody, &Radius, &mut Transform)>) {
    for _ in 0..SUBSTEPS {
        let dt = 1.0 / FREQUENCY / SUBSTEPS as f32 / TIME_SCALE;

        for (mut body, &Radius(radius), _) in query.iter_mut() {
            body.integrate(dt);

            if let Some(contact) = contact::contact(
                &Isometry::identity(),
                &HalfSpace::new(Unit::new_normalize(Vector::new(0.0, 1.0, 0.0))),
                &Isometry::from_subset(&Translation::new(
                    body.translation().x,
                    body.translation().y,
                    body.translation().z,
                )),
                &Ball { radius },
                0.0,
            )
            .unwrap()
            {
                body.push_impulse(STIFFNESS * contact.dist * -contact.normal1.to_superset());
            }
        }

        let mut combinations = query.iter_combinations_mut();
        while let Some([(mut b1, &Radius(r1), _), (mut b2, &Radius(r2), _)]) =
            combinations.fetch_next()
        {
            if let Some(contact) = contact::contact(
                &Isometry::from_subset(&Translation::new(
                    b1.translation().x,
                    b1.translation().y,
                    b1.translation().z,
                )),
                &Ball { radius: r1 },
                &Isometry::from_subset(&Translation::new(
                    b2.translation().x,
                    b2.translation().y,
                    b2.translation().z,
                )),
                &Ball { radius: r2 },
                0.0,
            )
            .unwrap()
            {
                b1.push_impulse(STIFFNESS * contact.dist * contact.normal1.to_superset());
                b2.push_impulse(STIFFNESS * contact.dist * -contact.normal1.to_superset());
            }
        }

        for (mut body, _, mut transform) in query.iter_mut() {
            body.apply_impulses();
            body.derive(dt);
            transform.translation = convert::vec(body.translation());
        }
    }
}

fn debug_bodies(query: Query<&RigidBody>, mut lines: ResMut<DebugLines>) {
    for body in query.iter() {
        lines.line_colored(
            convert::vec(body.translation()),
            convert::vec(body.translation() + body.velocity),
            0.0,
            Color::GREEN,
        );
    }
}
