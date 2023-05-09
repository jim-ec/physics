use cgmath::InnerSpace;

use crate::{
    algebra::{Pair, Vector},
    rigid_body::RigidBody,
};

/// The collider's center of mass coincides with the position of the rigid body it is attached to.
pub struct Ball {
    pub radius: f32,
}

pub struct Contact {
    pub points: Pair<Vector>,
    pub normals: Pair<Vector>,
}

fn test_collision(bodies: Pair<&RigidBody>, balls: Pair<&Ball>) -> Option<Contact> {
    let distance = (bodies.1.translation - bodies.0.translation).magnitude();
    todo!()
}
