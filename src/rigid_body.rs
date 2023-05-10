use bevy::prelude::*;
use derive_setters::*;

use crate::util::Vector;

#[derive(Debug, Component, Setters, Default, Clone, Copy)]
#[setters(generate_private = false)]
pub struct RigidBody {
    pub force: Vector,
    pub inverse_mass: f32,
    pub velocity: Vector,
    translation: Vector,
    next_translation: Vector,
    impulse: (Vector, usize),
}

impl RigidBody {
    pub fn teleport(mut self, translation: Vector) -> Self {
        self.translation = translation;
        self.next_translation = translation;
        self
    }

    pub fn translation(self) -> Vector {
        self.next_translation
    }

    pub fn integrate(&mut self, dt: f32) {
        self.velocity += dt * self.force * self.inverse_mass;
        self.next_translation = self.translation + dt * self.velocity;
    }

    pub fn derive(&mut self, dt: f32) {
        self.velocity = (self.next_translation - self.translation) / dt;
        self.translation = self.next_translation;
    }

    pub fn push_impulse(&mut self, impulse: Vector) {
        self.impulse.0 += impulse;
        self.impulse.1 += 1;
    }

    pub fn apply_impulses(&mut self) {
        if self.impulse.1 > 0 {
            let impulse = self.impulse.0 / self.impulse.1 as f32;
            self.next_translation += impulse * self.inverse_mass;
            self.impulse = (Vector::zeros(), 0);
        }
    }
}
