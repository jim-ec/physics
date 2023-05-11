use bevy::prelude::*;
use parry3d::shape::{Ball, Capsule, Segment, Shape};

use super::util::Point;

/// Collider component responsible for generating contacts.
#[derive(Component, Clone, Copy)]
pub enum Collider {
    #[allow(unused)]
    Ball { radius: f32 },
    #[allow(unused)]
    /// A line segment aligned with the Y-axis inflated by some radius.
    Capsule { radius: f32, length: f32 },
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
        }
    }
}
