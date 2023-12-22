/* For physics stuff, created 2023-12-16
This is where the `PhysicsController` trait is
*/

use rapier3d::prelude::*;
use crate::prelude::*;

#[allow(unused)]
pub fn defaut_extra_forces_calculator(lin: V3, ang: V3) -> BodyForces {
	BodyForces::default()
}

pub struct PhysicsUpdateArgs<'a> {
	pub dt: Float,
	pub gravity: Float,
	pub rapier: &'a mut PhysicsState,
	pub paths: &'a PathSet,
	pub latest_input: &'a Option<InputData>,
	pub extra_forces_calculator: &'a dyn Fn(V3, V3) -> BodyForces// (linear velocity, angular velocity) -> (linear force, torque); Both wrt body (not global)
}

pub trait PhysicsController {
	fn serializable(&self, bodies: &RigidBodySet, paths: &PathSet) -> BodyStateSerialize;
	fn update(&mut self, info: PhysicsUpdateArgs) -> BodyForces;
	fn recover_from_flip(&mut self, physics_state: &mut PhysicsState) {}
}

pub trait BodyAveragableState {
	fn average(&mut self, other: &Self);
}