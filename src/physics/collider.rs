use bevy::prelude::*;
use parry3d::na::Unit;

use super::{convert, util::Point};

#[derive(Component, Debug, Clone, Copy)]
pub struct Collider {
    // TODO: When calculating the volume of shapes is available, use enum with absolute mass or uniform density
    pub mass: f32,
    pub shape: Shape,
}

/// Collider component responsible for generating contacts.
#[derive(Component, Debug, Clone, Copy)]
pub enum Shape {
    #[allow(unused)]
    Ball { radius: f32 },
    #[allow(unused)]
    /// A line segment aligned with the Y-axis inflated by some radius.
    Capsule { radius: f32, length: f32 },
    #[allow(unused)]
    Plane { normal: Vec3 },
}

impl Collider {
    pub fn inv_mass(&self) -> f32 {
        self.mass.recip()
    }

    pub fn inv_moment_of_inertia(&self) -> Vec3 {
        let inverse_tensor = self
            .shape
            .parry_shape()
            .mass_properties(1.0) // TODO: Density is hardcoded here
            .inv_principal_inertia_sqrt
            .map(|v| v * v);
        convert::vec(inverse_tensor)
    }
}

impl Shape {
    pub(super) fn parry_shape(self) -> Box<dyn parry3d::shape::Shape> {
        match self {
            Shape::Ball { radius } => Box::new(parry3d::shape::Ball { radius }),
            Shape::Capsule { radius, length } => Box::new(parry3d::shape::Capsule {
                segment: parry3d::shape::Segment::new(
                    Point::new(0.0, length / 2.0, 0.0),
                    Point::new(0.0, -length / 2.0, 0.0),
                ),
                radius,
            }),
            Shape::Plane { normal } => Box::new(parry3d::shape::HalfSpace {
                normal: Unit::new_normalize(convert::to_vec(normal)),
            }),
        }
    }
}
