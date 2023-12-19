/* For physics stuff, created 2023-12-16
This is where the `PhysicsController` trait is
*/

use std::sync::Arc;
use rapier3d::prelude::*;
use crate::prelude::*;

pub struct PhysicsUpdateArgs<'a> {
	pub dt: Float,
	pub gravity: Float,
	pub rapier: &'a mut PhysicsState,
	pub paths: &'a PathSet,
	pub latest_input: &'a Option<InputData>
}

pub trait PhysicsController {
	fn build(
		body_state: &BodyStateSerialize,
		bodies: &mut RigidBodySet,
		colliders: &mut ColliderSet,
		joints: &mut ImpulseJointSet,
		paths: &PathSet,
		static_: Arc<VehicleStatic>
	) -> Self where Self: Sized + Send + Sync;
	fn serializable(&self, bodies: &RigidBodySet, paths: &PathSet) -> BodyStateSerialize;
	fn update(&mut self, info: PhysicsUpdateArgs);
	fn step(&mut self, info: PhysicsUpdateArgs);
	fn recover_from_flip(&mut self, physics_state: &mut PhysicsState);
}

pub trait BodyAveragableState {
	fn average(&mut self, other: &Self);
}