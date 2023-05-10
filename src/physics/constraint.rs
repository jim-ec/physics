#![allow(unused)]

use super::util::Vector;

#[derive(Debug)]
pub struct Constraint {
    pub rigid: usize,
    pub contacts: (Vector, Vector),
    pub distance: f32,
}
