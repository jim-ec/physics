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
use rigid_body::{RigidBody, RigidBodyBundle, RigidBodyIntegration};
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
        .add_system(integrate)
        .run()
}

pub const GRAVITY: f32 = 10.0;
pub const SUBSTEPS: usize = 10;
pub const STIFFNESS: f32 = 0.5;
pub const FREQUENCY: f32 = 60.0;
pub const TIME_SCALE: f32 = 1.0;
pub const CAMERA_DISTANCE: f32 = 20.0;

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
        commands.spawn((
            Radius(radius),
            RigidBodyBundle::from(RigidBody {
                force: Vec3::new(0.0, -GRAVITY, 0.0),
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

        height += 1.5;
    }
}

// TODO: Split into several sub-systems?
// TODO: Properly use global transform
fn integrate(
    mut query: Query<(
        &mut RigidBody,
        &mut RigidBodyIntegration,
        &Radius,
        &mut Transform,
    )>,
) {
    for _ in 0..SUBSTEPS {
        let dt = 1.0 / FREQUENCY / SUBSTEPS as f32 / TIME_SCALE;

        for (mut body, mut integration, &Radius(radius), transform) in query.iter_mut() {
            integration.integrate(&mut body, transform.translation, dt);

            if let Some(contact) = contact::contact(
                &Isometry::identity(),
                &HalfSpace::new(Unit::new_normalize(Vector::new(0.0, 1.0, 0.0))),
                &Isometry::from_subset(&Translation::new(
                    integration.translation().x,
                    integration.translation().y,
                    integration.translation().z,
                )),
                &Ball { radius },
                0.0,
            )
            .unwrap()
            {
                integration.push_impulse(
                    STIFFNESS * contact.dist * convert::vec(-contact.normal1.to_superset()),
                );
            }
        }

        let mut combinations = query.iter_combinations_mut();
        while let Some([(_, mut i1, &Radius(r1), _), (_, mut i2, &Radius(r2), _)]) =
            combinations.fetch_next()
        {
            if let Some(contact) = contact::contact(
                &Isometry::from_subset(&Translation::new(
                    i1.translation().x,
                    i1.translation().y,
                    i1.translation().z,
                )),
                &Ball { radius: r1 },
                &Isometry::from_subset(&Translation::new(
                    i2.translation().x,
                    i2.translation().y,
                    i2.translation().z,
                )),
                &Ball { radius: r2 },
                0.0,
            )
            .unwrap()
            {
                i1.push_impulse(
                    STIFFNESS * contact.dist * convert::vec(contact.normal1.to_superset()),
                );
                i2.push_impulse(
                    STIFFNESS * contact.dist * convert::vec(contact.normal2.to_superset()),
                );
            }
        }

        for (mut body, mut integration, _, mut transform) in query.iter_mut() {
            integration.apply_impulses(&body);
            transform.translation = integration.derive(&mut body, transform.translation, dt);
        }
    }
}

fn debug_bodies(query: Query<(&RigidBody, &Transform)>, mut lines: ResMut<DebugLines>) {
    for (body, transform) in query.iter() {
        lines.line_colored(
            transform.translation,
            transform.translation + body.velocity,
            0.0,
            Color::GREEN,
        );
    }
}
