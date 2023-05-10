use crate::{rigid_body::RigidBody, util::Vector};

#[derive(Debug)]
pub struct Constraint {
    pub rigid: usize,
    pub contacts: (Vector, Vector),
    pub distance: f32,
}
