pub mod collider;
pub mod rigid_body;

mod constraint;
mod convert;
mod util;

use bevy::prelude::*;
use bevy_prototype_debug_lines::{DebugLines, DebugLinesPlugin};
use parry3d::{
    math::Isometry, na::Unit, query::contact, shape::HalfSpace, simba::scalar::SubsetOf,
};

use self::{
    collider::Collider,
    rigid_body::{RigidBody, RigidBodyIntegration},
    util::Vector,
};

#[derive(Default, Debug)]
pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(DebugLinesPlugin::with_depth_test(false))
            .insert_resource(PhysicsParameters::default())
            .add_system(integrate)
            .add_system(debug_bodies);
    }
}

#[derive(Resource)]
pub struct PhysicsParameters {
    pub debug: bool,
    pub gravity: f32,
    pub substeps: usize,
    pub stiffness: f32,
    pub frequency: f32,
    pub time_scale: f32,
}

impl Default for PhysicsParameters {
    fn default() -> Self {
        Self {
            debug: false,
            gravity: 10.0,
            substeps: 10,
            stiffness: 1.0,
            frequency: 60.0,
            time_scale: 1.0,
        }
    }
}

// TODO: Split into several sub-systems? https://bevyengine.org/news/bevy-0-10/#managing-complex-control-flow-with-schedules
// TODO: Properly use global transform
fn integrate(
    mut query: Query<(
        &mut RigidBody,
        &mut RigidBodyIntegration,
        &Collider,
        &mut Transform,
    )>,
    parameters: Res<PhysicsParameters>,
    mut lines: ResMut<DebugLines>,
) {
    for _ in 0..parameters.substeps {
        let dt = 1.0 / parameters.frequency / parameters.substeps as f32 / parameters.time_scale;

        for (mut body, mut integration, collider, transform) in query.iter_mut() {
            let inertia_tensor = collider
                .parry_collider()
                .mass_properties(1.0)
                .principal_inertia();

            integration.integrate(
                &mut body,
                transform.translation,
                transform.rotation,
                Vec3::new(0.0, -parameters.gravity, 0.0),
                convert::vec(inertia_tensor),
                dt,
            );

            if let Some(contact) = contact::contact(
                &Isometry::identity(),
                &HalfSpace::new(Unit::new_normalize(Vector::new(0.0, 1.0, 0.0))),
                &convert::to_iso(Transform {
                    translation: integration.translation(),
                    rotation: integration.rotation(),
                    scale: Vec3::ONE,
                }),
                collider.parry_collider().as_ref(),
                0.0,
            )
            .unwrap()
            {
                integration.push_impulse(
                    convert::point(contact.point2),
                    parameters.stiffness
                        * contact.dist
                        * convert::vec(-contact.normal1.to_superset()),
                    &body,
                );

                debug_contact(&mut lines, contact, &parameters);
            }
        }

        let mut combinations = query.iter_combinations_mut();
        while let Some([(b1, mut i1, c1, _), (b2, mut i2, c2, _)]) = combinations.fetch_next() {
            if let Some(contact) = contact::contact(
                &convert::to_iso(Transform {
                    translation: i1.translation(),
                    rotation: i1.rotation(),
                    scale: Vec3::ONE,
                }),
                c1.parry_collider().as_ref(),
                &convert::to_iso(Transform {
                    translation: i2.translation(),
                    rotation: i2.rotation(),
                    scale: Vec3::ONE,
                }),
                c2.parry_collider().as_ref(),
                0.0,
            )
            .unwrap()
            {
                i1.push_impulse(
                    convert::point(contact.point1),
                    parameters.stiffness
                        * 0.5
                        * contact.dist
                        * convert::vec(contact.normal1.to_superset()),
                    &b1,
                );
                i2.push_impulse(
                    convert::point(contact.point2),
                    parameters.stiffness
                        * 0.5
                        * contact.dist
                        * convert::vec(contact.normal2.to_superset()),
                    &b2,
                );

                debug_contact(&mut lines, contact, &parameters);
            }
        }

        for (mut body, mut integration, _, mut transform) in query.iter_mut() {
            integration.apply_impulses(&body);
            integration.derive(&mut body, transform.translation, transform.rotation, dt);
            transform.translation = integration.translation();
            transform.rotation = integration.rotation();
        }
    }
}

// TODO: Use run condition: https://bevyengine.org/news/bevy-0-10/#run-conditions
fn debug_bodies(
    query: Query<(&RigidBody, &Transform)>,
    mut lines: ResMut<DebugLines>,
    parameters: Res<PhysicsParameters>,
) {
    if !parameters.debug {
        return;
    }

    for (body, transform) in query.iter() {
        lines.line_colored(
            transform.translation,
            transform.translation + body.velocity,
            0.0,
            Color::GREEN,
        );

        // lines.line_colored(
        //     transform.translation,
        //     Vec3::new(transform.translation.x, 0.0, transform.translation.z),
        //     0.0,
        //     Color::GRAY,
        // );
    }
}

// TODO: Use run condition: https://bevyengine.org/news/bevy-0-10/#run-conditions
fn debug_contact(
    lines: &mut ResMut<DebugLines>,
    contact: contact::Contact,
    parameters: &Res<PhysicsParameters>,
) {
    if parameters.debug {
        debug_point(lines, convert::point(contact.point1), Color::YELLOW);
        debug_point(lines, convert::point(contact.point2), Color::YELLOW);
        lines.line_colored(
            convert::point(contact.point1),
            convert::point(contact.point2),
            0.0,
            Color::YELLOW,
        );
    }
}

fn debug_point(lines: &mut ResMut<DebugLines>, p: Vec3, color: Color) {
    const L: f32 = 0.1;
    lines.line_colored(p + L * Vec3::X, p, 0.0, color);
    lines.line_colored(p - L * Vec3::X, p, 0.0, color);
    lines.line_colored(p + L * Vec3::Y, p, 0.0, color);
    lines.line_colored(p - L * Vec3::Y, p, 0.0, color);
    lines.line_colored(p + L * Vec3::Z, p, 0.0, color);
    lines.line_colored(p - L * Vec3::Z, p, 0.0, color);
}
