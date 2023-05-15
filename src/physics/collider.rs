use bevy::prelude::*;
use parry3d::{na::Unit, simba::scalar::SubsetOf};

use super::{convert, util::Point};

#[derive(Component, Debug)]
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

pub struct Contact {
    pub points: (Vec3, Vec3),
    pub normals: (Vec3, Vec3),
    pub depth: f32,
}

pub fn contact(
    colliders: (&Collider, &Collider),
    translations: (Vec3, Vec3),
    rotations: (Quat, Quat),
) -> Option<Contact> {
    if let Some(c) = parry3d::query::contact::contact(
        &convert::to_iso(Transform {
            translation: translations.0,
            rotation: rotations.0,
            scale: Vec3::ONE,
        }),
        colliders.0.shape.parry_shape().as_ref(),
        &convert::to_iso(Transform {
            translation: translations.1,
            rotation: rotations.1,
            scale: Vec3::ONE,
        }),
        colliders.1.shape.parry_shape().as_ref(),
        0.0,
    )
    .unwrap()
    {
        Some(Contact {
            points: (convert::point(c.point1), convert::point(c.point2)),
            normals: (
                convert::vec(c.normal1.to_superset()),
                convert::vec(c.normal2.to_superset()),
            ),
            depth: (c.dist),
        })
    } else {
        None
    }
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
    fn parry_shape(self) -> Box<dyn parry3d::shape::Shape> {
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
