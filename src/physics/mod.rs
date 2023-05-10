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
    shape::{Ball, HalfSpace},
    simba::scalar::{SubsetOf, SupersetOf},
};

use self::{
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

/// A proto-collider type.
#[derive(Component, Clone, Copy)]
pub struct Radius(pub f32);

// TODO: Split into several sub-systems?
// TODO: Properly use global transform
fn integrate(
    mut query: Query<(
        &mut RigidBody,
        &mut RigidBodyIntegration,
        &Radius,
        &mut Transform,
    )>,
    parameters: Res<PhysicsParameters>,
) {
    for _ in 0..parameters.substeps {
        let dt = 1.0 / parameters.frequency / parameters.substeps as f32 / parameters.time_scale;

        for (mut body, mut integration, &Radius(radius), transform) in query.iter_mut() {
            integration.integrate(
                &mut body,
                transform.translation,
                Vec3::new(0.0, -parameters.gravity, 0.0),
                dt,
            );

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
                    parameters.stiffness
                        * contact.dist
                        * convert::vec(-contact.normal1.to_superset()),
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
                    parameters.stiffness
                        * contact.dist
                        * convert::vec(contact.normal1.to_superset()),
                );
                i2.push_impulse(
                    parameters.stiffness
                        * contact.dist
                        * convert::vec(contact.normal2.to_superset()),
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
