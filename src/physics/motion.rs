use std::ops::Add;

use bevy::prelude::*;

/// A rigid body with linear motion.
// TODO: Add external and internal force
#[derive(Debug, Component, Clone, Copy)]
pub struct Linear {
    pub velocity: Vec3,
    pub(super) translation: Vec3,
    impulse: (Vec3, usize),
    mass: f32, // TODO: Store as inverse mass
}

/// A rigid body with angular motion.
// TODO: Add external and internal torque
#[derive(Debug, Component, Clone, Copy)]
pub struct Angular {
    pub angular_velocity: Vec3,
    pub(super) rotation: Quat,
    angular_impulse: (Vec3, usize),
    inverse_moment_of_inertia: Vec3,
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
            mass: f32::INFINITY,
        }
    }
}

impl Default for Angular {
    fn default() -> Self {
        Self {
            angular_velocity: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            angular_impulse: (Vec3::ZERO, 0),
            inverse_moment_of_inertia: Vec3::ZERO,
        }
    }
}

impl Linear {
    pub fn integrate(&mut self, translation: Vec3, force: Vec3, mass: f32, dt: f32) {
        self.velocity += dt * force / mass;
        self.translation = translation + dt * self.velocity;
        self.mass = mass;
    }

    pub fn push_impulse(&mut self, impulse: Vec3) {
        self.impulse.0 += impulse;
        self.impulse.1 += 1;
    }

    pub fn apply_impulses(&mut self) {
        if self.impulse.1 > 0 {
            let total_impulse = self.impulse.0 / self.impulse.1 as f32;
            self.translation += total_impulse / self.mass;
            self.impulse = (Vec3::ZERO, 0);
        }
    }

    pub fn derive(&mut self, translation: Vec3, dt: f32) {
        self.velocity = (self.translation - translation) / dt;
    }
}

impl Angular {
    pub fn integrate(&mut self, rotation: Quat, inverse_moment_of_inertia: Vec3, dt: f32) {
        let delta_rotation =
            Quat::from_vec4(dt * 0.5 * self.angular_velocity.extend(0.0)) * self.rotation;
        self.rotation = (rotation + delta_rotation).normalize();
        self.inverse_moment_of_inertia = inverse_moment_of_inertia;
    }

    pub fn push_impulse(&mut self, point_of_attack: Vec3, center_of_mass: Vec3, impulse: Vec3) {
        self.angular_impulse.0 +=
            (self.inverse_moment_of_inertia * (point_of_attack - center_of_mass)).cross(impulse);
        self.angular_impulse.1 += 1;
    }

    pub fn apply_impulses(&mut self) {
        if self.angular_impulse.1 > 0 {
            let delta = Quat::from_vec4(
                0.5 * self.angular_impulse.0.extend(0.0) / self.angular_impulse.1 as f32,
            ) * self.rotation;
            self.rotation = self.rotation.add(delta).normalize();
            self.angular_impulse = (Vec3::ZERO, 0);
        }
    }

    pub fn derive(&mut self, rotation: Quat, dt: f32) {
        let mut delta = self.rotation * rotation.conjugate();
        if delta.w < 0.0 {
            delta = -delta;
        }
        self.angular_velocity = 2.0 * delta.xyz() / dt;
    }
}
