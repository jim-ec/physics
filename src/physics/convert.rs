#![allow(unused)]

use bevy::prelude::{Quat, Transform, Vec3};
use parry3d::{
    math::{Isometry, Rotation},
    na::{Quaternion, UnitQuaternion},
    simba::scalar::SupersetOf,
};

use super::util::{Point, Translation, Vector};

pub fn vec(vec: Vector) -> Vec3 {
    Vec3::new(vec.x, vec.y, vec.z)
}

pub fn to_vec(vec: Vec3) -> Vector {
    Vector::new(vec.x, vec.y, vec.z)
}

pub fn point(vec: Point) -> Vec3 {
    Vec3::new(vec.x, vec.y, vec.z)
}

pub fn to_point(vec: Vec3) -> Point {
    Point::new(vec.x, vec.y, vec.z)
}

pub fn to_iso(f: Transform) -> Isometry<f32> {
    let t = f.translation;
    let r = f.rotation;
    Isometry {
        translation: Translation::new(t.x, t.y, t.z),
        rotation: UnitQuaternion::new_unchecked(Quaternion::new(r.w, r.x, r.y, r.z)),
    }
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
