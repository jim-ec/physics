use crate::util::Vector;

#[derive(Debug)]
pub struct RigidBody {
    pub force: Vector,
    pub inverse_mass: f32,
    pub velocity: Vector,
    pub translation: Vector,
}

impl RigidBody {
    pub fn integrate(&mut self, dt: f32) {
        self.velocity += dt * self.force * self.inverse_mass;
        self.translation += dt * self.velocity;
    }

    pub fn derive(&mut self, translation: Vector, dt: f32) {
        self.velocity = (self.translation - translation) / dt;
    }

    pub fn apply_impulse(&mut self, impulse: Vector) {
        self.translation += impulse * self.inverse_mass;
    }
}
