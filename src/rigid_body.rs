use bevy::prelude::*;
use derive_setters::*;

use crate::util::Vector;

// TODO: Use Bevy types
#[derive(Debug, Component, Clone, Copy)]
pub struct RigidBody {
    pub force: Vector,
    pub inverse_mass: f32, // TODO: Do not use inverse dimensionality
    pub velocity: Vector,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            force: Vector::zeros(),
            inverse_mass: 1.0,
            velocity: Vector::zeros(),
        }
    }
}

#[derive(Debug, Component, Default, Clone, Copy)]
pub struct RigidBodyIntegration {
    translation: Vector,
    impulse: (Vector, usize),
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
    pub fn integrate(&mut self, body: &mut RigidBody, translation: Vector, dt: f32) {
        body.velocity += dt * body.force * body.inverse_mass;
        self.translation = translation + dt * body.velocity;
    }

    pub fn translation(self) -> Vector {
        self.translation
    }

    pub fn push_impulse(&mut self, impulse: Vector) {
        self.impulse.0 += impulse;
        self.impulse.1 += 1;
    }

    pub fn apply_impulses(&mut self, body: &RigidBody) {
        if self.impulse.1 > 0 {
            let total_impulse = self.impulse.0 / self.impulse.1 as f32;
            self.translation += total_impulse * body.inverse_mass;
            self.impulse = (Vector::zeros(), 0);
        }
    }

    pub fn derive(&mut self, body: &mut RigidBody, translation: Vector, dt: f32) -> Vector {
        body.velocity = (self.translation - translation) / dt;
        self.translation
    }
}
