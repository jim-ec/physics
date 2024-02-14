pub mod collider;
pub mod motion;

mod constraint;
mod convert;
mod util;

use bevy::prelude::*;

use self::{
    collider::{contact, Collider, Contact},
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

        app.insert_resource(PhysicsParameters::default())
            .insert_resource(InternalParameters {
                substeps: self.substeps,
            })
            .add_systems(
                Update,
                debug_bodies.run_if(|param: Res<PhysicsParameters>| param.debug),
            );

        for _ in 0..self.substeps {
            app.add_systems(
                Update,
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
            collider.inv_mass(),
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
        angular.integrate(transform.rotation, collider.inv_moment_of_inertia(), dt);
    }
}

fn contacts(
    mut query: Query<(
        &Transform,
        &Collider,
        Option<&mut Linear>,
        Option<&mut Angular>,
    )>,
    parameters: Res<PhysicsParameters>,
    mut gizmos: Gizmos,
) {
    let mut combinations = query.iter_combinations_mut();
    while let Some(
        [(past_0, collider_0, mut linear_0, mut angular_0), (past_1, collider_1, mut linear_1, mut angular_1)],
    ) = combinations.fetch_next()
    {
        let translation_0 = match &linear_0 {
            Some(linear) => linear.translation,
            None => past_0.translation,
        };
        let translation_1 = match &linear_1 {
            Some(linear) => linear.translation,
            None => past_1.translation,
        };
        let rotation_0 = match &angular_0 {
            Some(angular) => angular.rotation,
            None => past_0.rotation,
        };
        let rotation_1 = match &angular_1 {
            Some(angular) => angular.rotation,
            None => past_1.rotation,
        };

        let _future_0 = Transform {
            translation: translation_0,
            rotation: rotation_0,
            scale: Vec3::ONE,
        };
        let _future_1 = Transform {
            translation: translation_1,
            rotation: rotation_1,
            scale: Vec3::ONE,
        };

        if let Some(contact) = contact(
            (collider_0, collider_1),
            (translation_0, translation_1),
            (rotation_0, rotation_1),
        ) {
            let mut _correction_1 = 0.5 * contact.depth * contact.normals.1;

            if let Some(linear) = &mut linear_0 {
                linear.push_impulse(parameters.stiffness * 0.5 * contact.depth * contact.normals.0);
            }
            if let Some(linear) = &mut linear_1 {
                linear.push_impulse(parameters.stiffness * 0.5 * contact.depth * contact.normals.1);
            }
            if let Some(angular) = &mut angular_0 {
                angular.push_impulse(
                    contact.points.0,
                    translation_0,
                    parameters.stiffness * 0.5 * contact.depth * contact.normals.0,
                );
            }
            if let Some(angular) = &mut angular_1 {
                angular.push_impulse(
                    contact.points.1,
                    translation_1,
                    parameters.stiffness * 0.5 * contact.depth * contact.normals.1,
                );
            }

            debug_contact(&mut gizmos, contact, &parameters);
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

fn debug_bodies(query: Query<(&Transform, Option<&Linear>)>, mut gizmos: Gizmos) {
    for (transform, linear) in query.iter() {
        if let Some(linear) = linear {
            gizmos.line(
                transform.translation,
                transform.translation + linear.velocity,
                Color::GREEN,
            );
        }

        gizmos.line(
            transform.translation,
            Vec3::new(transform.translation.x, 0.0, transform.translation.z),
            Color::GRAY,
        );
    }
}

fn debug_contact(gizmos: &mut Gizmos, contact: Contact, parameters: &Res<PhysicsParameters>) {
    if parameters.debug {
        debug_point(gizmos, contact.points.0, Color::YELLOW);
        debug_point(gizmos, contact.points.1, Color::YELLOW);
        gizmos.line(contact.points.0, contact.points.1, Color::YELLOW);
    }
}

fn debug_point(gizmos: &mut Gizmos, p: Vec3, color: Color) {
    const L: f32 = 0.1;
    gizmos.line(p + L * Vec3::X, p, color);
    gizmos.line(p - L * Vec3::X, p, color);
    gizmos.line(p + L * Vec3::Y, p, color);
    gizmos.line(p - L * Vec3::Y, p, color);
    gizmos.line(p + L * Vec3::Z, p, color);
    gizmos.line(p - L * Vec3::Z, p, color);
}
