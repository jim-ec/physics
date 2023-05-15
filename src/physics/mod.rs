pub mod collider;
pub mod motion;

mod constraint;
mod convert;
mod util;

use bevy::prelude::*;
use bevy_prototype_debug_lines::{DebugLines, DebugLinesPlugin};
use parry3d::{query::contact, simba::scalar::SubsetOf};

use self::{
    collider::Collider,
    motion::{Angular, Linear},
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
    mut query: Query<(&mut Linear, &Collider, &mut Transform)>,
    parameters: Res<PhysicsParameters>,
    internal: Res<InternalParameters>,
) {
    let dt = 1.0 / parameters.frequency / internal.substeps as f32 / parameters.time_scale;

    for (mut linear, collider, transform) in query.iter_mut() {
        linear.integrate(
            transform.translation,
            Vec3::new(0.0, -parameters.gravity, 0.0),
            collider.mass,
            dt,
        );
    }
}

fn integrate_rotation(
    mut query: Query<(&mut Angular, &Collider, &mut Transform)>,
    parameters: Res<PhysicsParameters>,
    internal: Res<InternalParameters>,
) {
    let dt = 1.0 / parameters.frequency / internal.substeps as f32 / parameters.time_scale;

    for (mut angular, collider, transform) in query.iter_mut() {
        angular.integrate(transform.rotation, collider.moment_of_inertia(), dt);
    }
}

fn contacts(
    mut query: Query<(Option<&mut Linear>, Option<&mut Angular>, &Collider)>,
    parameters: Res<PhysicsParameters>,
    mut lines: ResMut<DebugLines>,
) {
    let mut combinations = query.iter_combinations_mut();
    while let Some(
        [(mut linear_1, mut angular_1, collider_1), (mut linear_2, mut angular_2, collider_2)],
    ) = combinations.fetch_next()
    {
        let translation_1 = match &linear_1 {
            Some(linear) => linear.translation,
            None => Vec3::ZERO,
        };
        let translation_2 = match &linear_2 {
            Some(linear) => linear.translation,
            None => Vec3::ZERO,
        };
        let rotation_1 = match &angular_1 {
            Some(angular) => angular.rotation,
            None => Quat::IDENTITY,
        };
        let rotation_2 = match &angular_2 {
            Some(angular) => angular.rotation,
            None => Quat::IDENTITY,
        };

        if let Some(contact) = contact::contact(
            &convert::to_iso(Transform {
                translation: translation_1,
                rotation: rotation_1,
                scale: Vec3::ONE,
            }),
            collider_1.shape.parry_shape().as_ref(),
            &convert::to_iso(Transform {
                translation: translation_2,
                rotation: rotation_2,
                scale: Vec3::ONE,
            }),
            collider_2.shape.parry_shape().as_ref(),
            0.0,
        )
        .unwrap()
        {
            if let Some(linear) = &mut linear_1 {
                linear.push_impulse(
                    parameters.stiffness
                        * 0.5
                        * contact.dist
                        * convert::vec(contact.normal1.to_superset()),
                );
            }
            if let Some(linear) = &mut linear_2 {
                linear.push_impulse(
                    parameters.stiffness
                        * 0.5
                        * contact.dist
                        * convert::vec(contact.normal2.to_superset()),
                );
            }
            if let Some(angular) = &mut angular_1 {
                angular.push_impulse(
                    convert::point(contact.point1),
                    translation_1,
                    parameters.stiffness
                        * 0.5
                        * contact.dist
                        * convert::vec(contact.normal1.to_superset()),
                );
            }
            if let Some(angular) = &mut angular_2 {
                angular.push_impulse(
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
    mut query: Query<(&mut Linear, &mut Transform)>,
    parameters: Res<PhysicsParameters>,
    internal: Res<InternalParameters>,
) {
    let dt = 1.0 / parameters.frequency / internal.substeps as f32 / parameters.time_scale;

    for (mut linear, mut transform) in query.iter_mut() {
        linear.apply_impulses();
        linear.derive(transform.translation, dt);
        transform.translation = linear.translation;
    }
}

fn derive_rotation(
    mut query: Query<(&mut Angular, &mut Transform)>,
    parameters: Res<PhysicsParameters>,
    internal: Res<InternalParameters>,
) {
    let dt = 1.0 / parameters.frequency / internal.substeps as f32 / parameters.time_scale;

    for (mut angular, mut transform) in query.iter_mut() {
        angular.apply_impulses();
        angular.derive(transform.rotation, dt);
        transform.rotation = angular.rotation;
    }
}

fn debug_bodies(query: Query<(&Transform, Option<&Linear>)>, mut lines: ResMut<DebugLines>) {
    for (transform, linear) in query.iter() {
        if let Some(linear) = linear {
            lines.line_colored(
                transform.translation,
                transform.translation + linear.velocity,
                0.0,
                Color::GREEN,
            );
        }

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
