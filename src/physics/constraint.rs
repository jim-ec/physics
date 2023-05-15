use bevy::prelude::*;

#[derive(Debug)]
pub struct PositionalConstraint {
    pub bodies: (Entity, Entity),
    pub contacts: (Vec3, Vec3),
    pub distance: f32,
}
