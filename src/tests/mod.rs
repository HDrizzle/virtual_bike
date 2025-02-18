use std::{collections::HashMap, sync::Arc};
#[cfg(any(feature = "server", feature = "client"))]
use std::rc::Rc;
use approx::assert_relative_eq;
use crate::prelude::*;

pub mod paths;

#[cfg(feature = "server")]
pub mod gen;

use super::*;
#[test]
fn rounding() {
	assert_eq!(round_float_towards_neg_inf(3.5), 3);
	assert_eq!(round_float_towards_neg_inf(0.0), 0);
	assert_eq!(round_float_towards_neg_inf(-0.0), 0);
	assert_eq!(round_float_towards_neg_inf(-3.0), -3);
	assert_eq!(round_float_towards_neg_inf(-3.5), -4);
}
#[test]
fn fancy_modulus() {
	assert_eq!(mod_or_clamp(5, 10, true), (5, false));
	assert_eq!(mod_or_clamp(5, 10, false), (5, false));
	assert_eq!(mod_or_clamp(0, 10, false), (0, false));
	assert_eq!(mod_or_clamp(9, 10, false), (9, false));
	assert_eq!(mod_or_clamp(10, 10, false), (9, true));
	assert_eq!(mod_or_clamp(-1, 10, false), (0, true));
	assert_eq!(mod_or_clamp(10, 10, true), (0, true));
	assert_eq!(mod_or_clamp(-1, 10, true), (9, true));
}
#[test]
#[should_panic]
fn rel_eq_test() {
	assert_relative_eq!(EPSILON * 2.0, 0.0, epsilon = EPSILON);// Just to make sure I'm using this right
}
#[test]
fn client_and_server_features_enabled() {
	#[cfg(not(all(feature = "server", feature = "client")))]
	panic!("Both the `server` and `client` features should be enabled for testing");
}
// TODO
/*#[test]
fn simple_rotation_composition() {
	let quat = UnitQuaternion::<Float>::from_axis_angle(&V3::y_axis(), PI/4.0);// Quat must not have roll
	assert_relative_eq!(SimpleRotation::from_quat(&quat).to_quat(), quat);
}*/