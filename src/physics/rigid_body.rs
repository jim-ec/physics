use std::ops::Add;

use bevy::prelude::*;

// TODO: Store inertia tensor
#[derive(Debug, Component, Clone, Copy)]
pub struct RigidBody {
    pub mass: f32, // TODO: Use enum with absolute mass or density
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            mass: 1.0,
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        }
    }
}

// TODO: Split into translational and rotational components and bundle
#[derive(Debug, Component, Default, Clone, Copy)]
pub struct RigidBodyIntegration {
    translation: Vec3,
    rotation: Quat,
    impulse: (Vec3, usize),
    angular_impulse: (Vec3, usize),
    inverse_inertia_tensor: Vec3,
}

#[derive(Debug, Bundle, Default, Clone, Copy)]
pub struct RigidBodyBundle {
    body: RigidBody,
    integration: RigidBodyIntegration,
}

impl From<RigidBody> for RigidBodyBundle {
    fn from(body: RigidBody) -> Self {
        RigidBodyBundle {
            body,
            integration: RigidBodyIntegration::default(),
        }
    }
}

impl RigidBodyIntegration {
    pub fn integrate(
        &mut self,
        body: &mut RigidBody,
        translation: Vec3,
        rotation: Quat,
        force: Vec3,
        inverse_inertia_tensor: Vec3,
        dt: f32,
    ) {
        body.velocity += dt * force / body.mass;
        self.translation = translation + dt * body.velocity;

        let delta_rotation =
            Quat::from_vec4(dt * 0.5 * body.angular_velocity.extend(0.0)) * self.rotation;
        self.rotation = (rotation + delta_rotation).normalize();

        self.inverse_inertia_tensor = inverse_inertia_tensor;
    }

    pub fn translation(self) -> Vec3 {
        self.translation
    }

    pub fn rotation(self) -> Quat {
        self.rotation
    }

    pub fn push_impulse(&mut self, point_of_attack: Vec3, impulse: Vec3) {
        self.impulse.0 += impulse;
        self.impulse.1 += 1;

        self.angular_impulse.0 +=
            (self.inverse_inertia_tensor * (point_of_attack - self.translation)).cross(impulse);
        self.angular_impulse.1 += 1;
    }

    pub fn apply_impulses(&mut self, body: &RigidBody) {
        if self.impulse.1 > 0 {
            let total_impulse = self.impulse.0 / self.impulse.1 as f32;
            self.translation += total_impulse / body.mass;
            self.impulse = (Vec3::ZERO, 0);
        }
        if self.angular_impulse.1 > 0 {
            let delta = Quat::from_vec4(
                0.5 * self.angular_impulse.0.extend(0.0) / self.angular_impulse.1 as f32,
            ) * self.rotation;
            self.rotation = self.rotation.add(delta).normalize();
            self.angular_impulse = (Vec3::ZERO, 0);
        }
    }

    pub fn derive(&mut self, body: &mut RigidBody, translation: Vec3, rotation: Quat, dt: f32) {
        body.velocity = (self.translation - translation) / dt;

        let mut delta = self.rotation * rotation.conjugate();
        if delta.w < 0.0 {
            delta = -delta;
        }
        body.angular_velocity = 2.0 * delta.xyz() / dt;
    }
}
