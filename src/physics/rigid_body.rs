use bevy::prelude::*;

#[derive(Debug, Component, Clone, Copy)]
pub struct RigidBody {
    pub mass: f32,
    pub velocity: Vec3,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            mass: 1.0,
            velocity: Vec3::ZERO,
        }
    }
}

// TODO: Split into translational and rotational components and bundle
#[derive(Debug, Component, Default, Clone, Copy)]
pub struct RigidBodyIntegration {
    // Currently, this stores the final value after the integration.
    // TODO: Store delta-value instead?
    // Probably not possible because apply_impulse_at() requires world coordinates.
    translation: Vec3,
    rotation: Quat,
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
    pub fn integrate(
        &mut self,
        body: &mut RigidBody,
        translation: Vec3,
        rotation: Quat,
        force: Vec3,
        dt: f32,
    ) {
        body.velocity += dt * force / body.mass;
        self.translation = translation + dt * body.velocity;

        self.rotation = rotation;
    }

    pub fn translation(self) -> Vec3 {
        self.translation
    }

    pub fn rotation(self) -> Quat {
        self.rotation
    }

    pub fn push_impulse(&mut self, impulse: Vec3, body: &RigidBody) {
        self.translation += impulse / body.mass;
        self.impulse = (Vec3::ZERO, 0);
        // self.impulse.0 += impulse;
        // self.impulse.1 += 1;
    }

    pub fn apply_impulses(&mut self, body: &RigidBody) {
        let _ = body;
        // if self.impulse.1 > 0 {
        //     let total_impulse = self.impulse.0 / self.impulse.1 as f32;
        //     self.translation += total_impulse / body.mass;
        //     self.impulse = (Vec3::ZERO, 0);
        // }
    }

    pub fn derive(&mut self, body: &mut RigidBody, translation: Vec3, dt: f32) -> Vec3 {
        body.velocity = (self.translation - translation) / dt;
        self.translation
    }
}
