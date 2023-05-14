use bevy::prelude::*;
use parry3d::{
    na::Unit,
    shape::{Ball, Capsule, HalfSpace, Segment, Shape},
};

use super::{convert, util::Point};

/// Collider component responsible for generating contacts.
#[derive(Component, Debug, Clone, Copy)]
pub enum Collider {
    #[allow(unused)]
    Ball { radius: f32 },
    #[allow(unused)]
    /// A line segment aligned with the Y-axis inflated by some radius.
    Capsule { radius: f32, length: f32 },
    #[allow(unused)]
    Plane { normal: Vec3 },
}

impl Collider {
    pub(super) fn parry_collider(self) -> Box<dyn Shape> {
        match self {
            Collider::Ball { radius } => Box::new(Ball { radius }),
            Collider::Capsule { radius, length } => Box::new(Capsule {
                segment: Segment::new(
                    Point::new(0.0, length / 2.0, 0.0),
                    Point::new(0.0, -length / 2.0, 0.0),
                ),
                radius,
            }),
            Collider::Plane { normal } => Box::new(HalfSpace {
                normal: Unit::new_normalize(convert::to_vec(normal)),
            }),
        }
    }
}
