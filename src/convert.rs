#![allow(unused)]

use bevy::prelude::{Quat, Transform, Vec3};
use parry3d::math::{Isometry, Point};

use crate::util::Vector;

pub fn vec(vec: Vector) -> Vec3 {
    Vec3::new(vec.x, vec.y, vec.z)
}

pub fn point(vec: Point<f32>) -> Vec3 {
    Vec3::new(vec.x, vec.y, vec.z)
}

pub fn iso(iso: Isometry<f32>) -> Transform {
    Transform {
        translation: Vec3 {
            x: iso.translation.x,
            y: iso.translation.y,
            z: iso.translation.z,
        },
        rotation: Quat {
            x: iso.rotation.i,
            y: iso.rotation.j,
            z: iso.rotation.k,
            w: iso.rotation.w,
        },
        scale: Vec3::ONE,
    }
}
