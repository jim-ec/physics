pub mod collider;
pub mod rigid_body;

mod constraint;
mod convert;
mod util;

use bevy::prelude::*;
use bevy_prototype_debug_lines::{DebugLines, DebugLinesPlugin};
use parry3d::{query::contact, simba::scalar::SubsetOf};

use self::{
    collider::Collider,
    rigid_body::{RigidBody, Rotational, Translational},
};

#[derive(Debug)]
pub struct PhysicsPlugin {
    pub substeps: usize,
}

impl Default for PhysicsPlugin {
    fn default() -> Self {
        Self { substeps: 10 }
    }
}

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        // TODO: Fix order dependencies using system sets

        app.add_plugin(DebugLinesPlugin::with_depth_test(false))
            .insert_resource(PhysicsParameters::default())
            .insert_resource(InternalParameters {
                substeps: self.substeps,
            })
            .add_system(debug_bodies.run_if(|param: Res<PhysicsParameters>| param.debug));

        for _ in 0..self.substeps {
            app.add_systems(
                (
                    integrate_translation,
                    integrate_rotation,
                    contacts,
                    derive_translation,
                    derive_rotation,
                )
                    .chain(),
            );
        }
    }
}

#[derive(Resource)]
struct InternalParameters {
    substeps: usize,
}

#[derive(Resource)]
pub struct PhysicsParameters {
    pub debug: bool,
    pub gravity: f32,
    pub stiffness: f32,
    pub frequency: f32,
    pub time_scale: f32,
}

impl Default for PhysicsParameters {
    fn default() -> Self {
        Self {
            debug: false,
            gravity: 10.0,
            stiffness: 1.0,
            frequency: 60.0,
            time_scale: 1.0,
        }
    }
}

fn integrate_translation(
    mut query: Query<(&mut RigidBody, &mut Translational, &mut Transform)>,
    parameters: Res<PhysicsParameters>,
    internal: Res<InternalParameters>,
) {
    let dt = 1.0 / parameters.frequency / internal.substeps as f32 / parameters.time_scale;

    for (mut body, mut translational, transform) in query.iter_mut() {
        translational.integrate(
            &mut body,
            transform.translation,
            Vec3::new(0.0, -parameters.gravity, 0.0),
            dt,
        );
    }
}

fn integrate_rotation(
    mut query: Query<(&mut RigidBody, &mut Rotational, &Collider, &mut Transform)>,
    parameters: Res<PhysicsParameters>,
    internal: Res<InternalParameters>,
) {
    let dt = 1.0 / parameters.frequency / internal.substeps as f32 / parameters.time_scale;

    for (mut body, mut rotational, collider, transform) in query.iter_mut() {
        let inverse_inertia_tensor = collider
            .parry_collider()
            .mass_properties(1.0)
            .inv_principal_inertia_sqrt
            .map(|v| v * v);
        rotational.integrate(
            &mut body,
            transform.rotation,
            convert::vec(inverse_inertia_tensor),
            dt,
        );
    }
}

fn contacts(
    mut query: Query<(
        Option<&mut Translational>,
        Option<&mut Rotational>,
        &Collider,
    )>,
    parameters: Res<PhysicsParameters>,
    mut lines: ResMut<DebugLines>,
) {
    let mut combinations = query.iter_combinations_mut();
    while let Some(
        [(mut translational_1, mut rotational_1, collider_1), (mut translational_2, mut rotational_2, collider_2)],
    ) = combinations.fetch_next()
    {
        let translation_1 = match &translational_1 {
            Some(translational) => translational.translation(),
            None => Vec3::ZERO,
        };
        let translation_2 = match &translational_2 {
            Some(translational) => translational.translation(),
            None => Vec3::ZERO,
        };
        let rotation_1 = match &rotational_1 {
            Some(rotational) => rotational.rotation(),
            None => Quat::IDENTITY,
        };
        let rotation_2 = match &rotational_2 {
            Some(rotational) => rotational.rotation(),
            None => Quat::IDENTITY,
        };

        if let Some(contact) = contact::contact(
            &convert::to_iso(Transform {
                translation: translation_1,
                rotation: rotation_1,
                scale: Vec3::ONE,
            }),
            collider_1.parry_collider().as_ref(),
            &convert::to_iso(Transform {
                translation: translation_2,
                rotation: rotation_2,
                scale: Vec3::ONE,
            }),
            collider_2.parry_collider().as_ref(),
            0.0,
        )
        .unwrap()
        {
            if let Some(translational) = &mut translational_1 {
                translational.push_impulse(
                    parameters.stiffness
                        * 0.5
                        * contact.dist
                        * convert::vec(contact.normal1.to_superset()),
                );
            }
            if let Some(translational) = &mut translational_2 {
                translational.push_impulse(
                    parameters.stiffness
                        * 0.5
                        * contact.dist
                        * convert::vec(contact.normal2.to_superset()),
                );
            }
            if let Some(rotational) = &mut rotational_1 {
                rotational.push_impulse(
                    convert::point(contact.point1),
                    translation_1,
                    parameters.stiffness
                        * 0.5
                        * contact.dist
                        * convert::vec(contact.normal1.to_superset()),
                );
            }
            if let Some(rotational) = &mut rotational_2 {
                rotational.push_impulse(
                    convert::point(contact.point2),
                    translation_2,
                    parameters.stiffness
                        * 0.5
                        * contact.dist
                        * convert::vec(contact.normal2.to_superset()),
                );
            }

            debug_contact(&mut lines, contact, &parameters);
        }
    }
}

fn derive_translation(
    mut query: Query<(&mut RigidBody, &mut Translational, &mut Transform)>,
    parameters: Res<PhysicsParameters>,
    internal: Res<InternalParameters>,
) {
    let dt = 1.0 / parameters.frequency / internal.substeps as f32 / parameters.time_scale;

    for (mut body, mut translational, mut transform) in query.iter_mut() {
        translational.apply_impulses(&body);
        translational.derive(&mut body, transform.translation, dt);
        transform.translation = translational.translation();
    }
}

fn derive_rotation(
    mut query: Query<(&mut RigidBody, &mut Rotational, &mut Transform)>,
    parameters: Res<PhysicsParameters>,
    internal: Res<InternalParameters>,
) {
    let dt = 1.0 / parameters.frequency / internal.substeps as f32 / parameters.time_scale;

    for (mut body, mut rotational, mut transform) in query.iter_mut() {
        rotational.apply_impulses();
        rotational.derive(&mut body, transform.rotation, dt);
        transform.rotation = rotational.rotation();
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
