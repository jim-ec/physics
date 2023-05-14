pub mod collider;
pub mod rigid_body;

mod constraint;
mod convert;
mod util;

use bevy::prelude::*;
use bevy_prototype_debug_lines::{DebugLines, DebugLinesPlugin};
use parry3d::{
    math::Isometry,
    na::Unit,
    query::contact,
    shape::HalfSpace,
    simba::scalar::{SubsetOf, SupersetOf},
};

use self::{
    collider::Collider,
    rigid_body::{RigidBody, RigidBodyIntegration},
    util::{Translation, Vector},
};

#[derive(Default, Debug)]
pub struct PhysicsPlugin {
    pub debug: bool,
}

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(DebugLinesPlugin::with_depth_test(false))
            .insert_resource(PhysicsParameters::default())
            .add_system(integrate);

        if self.debug {
            app.add_system(debug_bodies);
        }
    }
}

#[derive(Resource)]
pub struct PhysicsParameters {
    pub gravity: f32,
    pub substeps: usize,
    pub stiffness: f32,
    pub frequency: f32,
    pub time_scale: f32,
}

impl Default for PhysicsParameters {
    fn default() -> Self {
        Self {
            gravity: 10.0,
            substeps: 10,
            stiffness: 1.0,
            frequency: 60.0,
            time_scale: 1.0,
        }
    }
}

// TODO: Split into several sub-systems?
// TODO: Properly use global transform
fn integrate(
    mut query: Query<(
        &mut RigidBody,
        &mut RigidBodyIntegration,
        &Collider,
        &mut Transform,
    )>,
    parameters: Res<PhysicsParameters>,
) {
    for _ in 0..parameters.substeps {
        let dt = 1.0 / parameters.frequency / parameters.substeps as f32 / parameters.time_scale;

        for (mut body, mut integration, collider, transform) in query.iter_mut() {
            integration.integrate(
                &mut body,
                transform.translation,
                transform.rotation,
                Vec3::new(0.0, -parameters.gravity, 0.0),
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
                    parameters.stiffness
                        * contact.dist
                        * convert::vec(-contact.normal1.to_superset()),
                    &body,
                );
            }
        }

        let mut combinations = query.iter_combinations_mut();
        while let Some([(b1, mut i1, c1, _), (b2, mut i2, c2, _)]) = combinations.fetch_next() {
            if let Some(contact) = contact::contact(
                &Isometry::from_subset(&Translation::new(
                    i1.translation().x,
                    i1.translation().y,
                    i1.translation().z,
                )),
                c1.parry_collider().as_ref(),
                &Isometry::from_subset(&Translation::new(
                    i2.translation().x,
                    i2.translation().y,
                    i2.translation().z,
                )),
                c2.parry_collider().as_ref(),
                0.0,
            )
            .unwrap()
            {
                i1.push_impulse(
                    parameters.stiffness
                        * contact.dist
                        * convert::vec(contact.normal1.to_superset()),
                    &b1,
                );
                i2.push_impulse(
                    parameters.stiffness
                        * contact.dist
                        * convert::vec(contact.normal2.to_superset()),
                    &b2,
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

        lines.line_colored(
            transform.translation,
            Vec3::new(transform.translation.x, 0.0, transform.translation.z),
            0.0,
            Color::GRAY,
        );
    }
}
