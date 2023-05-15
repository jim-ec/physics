use std::ops::Add;

use bevy::prelude::*;
use derive_setters::Setters;

/// A rigid body with linear motion.
// TODO: Add external and internal force
#[derive(Debug, Component, Clone, Copy, Setters)]
#[setters(generate_private = false)]
pub struct Linear {
    pub velocity: Vec3,
    pub(super) translation: Vec3,
    impulse: (Vec3, usize),
    inv_mass: f32,
}

/// A rigid body with angular motion.
// TODO: Add external and internal torque
#[derive(Debug, Component, Clone, Copy, Setters)]
#[setters(generate_private = false)]
pub struct Angular {
    pub angular_velocity: Vec3,
    pub(super) rotation: Quat,
    angular_impulse: (Vec3, usize),
    inv_moment_of_inertia: Vec3,
}

/// A rigid body combines linear and angular motion.
#[derive(Debug, Bundle, Default, Clone, Copy)]
pub struct Rigid {
    linear: Linear,
    angular: Angular,
}

impl Default for Linear {
    fn default() -> Self {
        Self {
            velocity: Vec3::ZERO,
            translation: Vec3::ZERO,
            impulse: (Vec3::ZERO, 0),
            inv_mass: 0.0,
        }
    }
}

impl Default for Angular {
    fn default() -> Self {
        Self {
            angular_velocity: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            angular_impulse: (Vec3::ZERO, 0),
            inv_moment_of_inertia: Vec3::ZERO,
        }
    }
}

impl Linear {
    pub(super) fn integrate(&mut self, translation: Vec3, force: Vec3, inverse_mass: f32, dt: f32) {
        self.velocity += dt * force * inverse_mass;
        self.translation = translation + dt * self.velocity;
        self.inv_mass = inverse_mass;
    }

    pub(super) fn push_impulse(&mut self, impulse: Vec3) {
        self.impulse.0 += impulse;
        self.impulse.1 += 1;
    }

    pub(super) fn apply_impulses(&mut self) {
        if self.impulse.1 > 0 {
            let total_impulse = self.impulse.0 / self.impulse.1 as f32;
            self.translation += total_impulse * self.inv_mass;
            self.impulse = (Vec3::ZERO, 0);
        }
    }

    pub(super) fn derive(&mut self, translation: Vec3, dt: f32) {
        self.velocity = (self.translation - translation) / dt;
    }
}

impl Angular {
    pub(super) fn integrate(&mut self, rotation: Quat, inverse_moment_of_inertia: Vec3, dt: f32) {
        let delta_rotation =
            Quat::from_vec4(dt * 0.5 * self.angular_velocity.extend(0.0)) * self.rotation;
        self.rotation = (rotation + delta_rotation).normalize();
        self.inv_moment_of_inertia = inverse_moment_of_inertia;
    }

    pub(super) fn push_impulse(
        &mut self,
        point_of_attack: Vec3,
        center_of_mass: Vec3,
        impulse: Vec3,
    ) {
        self.angular_impulse.0 +=
            (self.inv_moment_of_inertia * (point_of_attack - center_of_mass)).cross(impulse);
        self.angular_impulse.1 += 1;
    }

    pub(super) fn apply_impulses(&mut self) {
        if self.angular_impulse.1 > 0 {
            let delta = Quat::from_vec4(
                0.5 * self.angular_impulse.0.extend(0.0) / self.angular_impulse.1 as f32,
            ) * self.rotation;
            self.rotation = self.rotation.add(delta).normalize();
            self.angular_impulse = (Vec3::ZERO, 0);
        }
    }

    pub(super) fn derive(&mut self, rotation: Quat, dt: f32) {
        let mut delta = self.rotation * rotation.conjugate();
        if delta.w < 0.0 {
            delta = -delta;
        }
        self.angular_velocity = 2.0 * delta.xyz() / dt;
    }
}
