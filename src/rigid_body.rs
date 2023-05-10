use bevy::prelude::*;

#[derive(Debug, Component, Clone, Copy)]
pub struct RigidBody {
    pub force: Vec3,
    pub mass: f32,
    pub velocity: Vec3,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            force: Vec3::ZERO,
            mass: 1.0,
            velocity: Vec3::ZERO,
        }
    }
}

#[derive(Debug, Component, Default, Clone, Copy)]
pub struct RigidBodyIntegration {
    translation: Vec3,
    impulse: (Vec3, usize),
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
    pub fn integrate(&mut self, body: &mut RigidBody, translation: Vec3, dt: f32) {
        body.velocity += dt * body.force / body.mass;
        self.translation = translation + dt * body.velocity;
    }

    pub fn translation(self) -> Vec3 {
        self.translation
    }

    pub fn push_impulse(&mut self, impulse: Vec3) {
        self.impulse.0 += impulse;
        self.impulse.1 += 1;
    }

    pub fn apply_impulses(&mut self, body: &RigidBody) {
        if self.impulse.1 > 0 {
            let total_impulse = self.impulse.0 / self.impulse.1 as f32;
            self.translation += total_impulse / body.mass;
            self.impulse = (Vec3::ZERO, 0);
        }
    }

    pub fn derive(&mut self, body: &mut RigidBody, translation: Vec3, dt: f32) -> Vec3 {
        body.velocity = (self.translation - translation) / dt;
        self.translation
    }
}
