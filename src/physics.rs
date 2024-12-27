//! For physics stuff, created 2023-12-16
//! This is where the `PhysicsController` trait is

use crate::prelude::*;

#[allow(unused)]
pub fn defaut_extra_forces_calculator(lin: V3, ang: V3) -> BodyForces {
	BodyForces::default()
}

pub struct PhysicsUpdateArgs<'a> {
	/// Delta-time
	pub dt: Float,
	/// Gravity, along 3D Y axis
	pub gravity: Float,
	/// Path set
	pub path_set: &'a PathSet,
	/// Latest user input
	pub latest_input: &'a Option<InputData>,
	/// (linear velocity, angular velocity) -> (linear force, torque); Both wrt body (not global)
	pub drag_force_calculator: &'a dyn Fn(V3, V3) -> BodyForces
}

/// Generalization over rapier and paths
pub trait PhysicsController {
	fn serializable(&self, paths: &PathSet) -> BodyStateSerialize;
	fn update(&mut self, info: PhysicsUpdateArgs) -> (BodyForces, Option<PathBodyForceDescription>);
	fn get_route_opt(&self, routes: &GenericDataset<Route>) -> Option<GenericRef<Route>>;
}

/// Currently not used, but may be used for 2/3 numerical integration stepping
pub trait BodyAveragableState {
	/// Averages `self` with `other` in-place
	fn average(&mut self, other: &Self);
}

/// For calculating drag force
pub fn fluid_drag_force(density: Float, speed: Float, cd: Float, cross_sectional_area: Float) -> Float {
	0.5 * density * speed*speed * cd * cross_sectional_area
}