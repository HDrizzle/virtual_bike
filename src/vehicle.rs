/* Defines a vehicle in the game
Potentially usefull https://www.sheldonbrown.com/rinard/aero/formulas.html
Any vehicle can be in one of two states: normal physics and path-bound. Normal physics means it will simply be controlled by the physics engine (Rapier 3D).
when path-bound it will be controlled by the specific Path it is bound to. it's velocity will be a single scalar value whose sign indicates which direction it is travelling on the path.
*/

use std::{error::Error, sync::Arc};
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
#[cfg(feature = "frontend")]
use bevy::{prelude::*, ecs::component::Component};
#[cfg(feature = "frontend")]
#[cfg(feature = "debug_render_physics")]
use bevy_rapier3d::plugin::RapierContext;
use nalgebra::{vector, point, UnitQuaternion};

// Rapier 3D physics
use rapier3d::{prelude::*, math::Real};
//use nalgebra::{Isometry3, Vector3, vector, Point3, point, geometry::Quaternion, UnitQuaternion, Const};

use crate::{prelude::*, map::path::PathBoundBodyState};

const STATIONARY_DRIVE_ACC_LIMIT: Float = 10.0;// m/s^2

#[derive(Serialize, Deserialize, Clone)]
pub struct WheelStatic {
	pub dia: Float,
	pub width: Float,
	pub friction: Float,
	pub position_wrt_body: Iso,// Relative to the vehicle body
	pub driven: bool,
	pub steering: bool,
	pub steering_rotation_axis_offset: Float, // A non-zero value of this will result in something similar to a shopping-cart wheel
}

impl WheelStatic {
	pub fn build_rapier_collider(&self) -> Collider {
		ColliderBuilder::cylinder((self.width / 2.0) as Real, (self.dia / 2.0) as Real)
			.restitution(0.9)
			.position(/*Iso::new(Translation{vector: V3::new(0., 0., 0.)}, UnitQuaternion::new(V3::x()))*/
				Iso {
					rotation: UnitQuaternion::new(V3::z() * std::f32::consts::FRAC_PI_2),// https://docs.rs/nalgebra/0.32.1/nalgebra/geometry/type.UnitQuaternion.html#method.from_axis_angle
					translation: Translation{vector: V3::new(0., 0., 0.)}
				}
			)// https://rapier.rs/docs/user_guides/rust/colliders#position
			.friction(self.friction)
			.build()
	}
}

type WheelSteering = Option<(RigidBodyHandle, ImpulseJointHandle)>;

#[derive(Clone)]
#[cfg_attr(feature = "frontend", derive(Component))]
pub struct Wheel {
	static_: WheelStatic,
	body_handle: RigidBodyHandle,
	collider_handle: ColliderHandle,
	steering: WheelSteering// None if not static_.steering or built in client
}

impl Wheel {
	pub fn build(static_: &WheelStatic, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, joints: &mut ImpulseJointSet, v_body_handle: RigidBodyHandle) -> Self {
		// Get vehicle position
		let vehicle_position = bodies.get(v_body_handle).expect("Unable to get vehicle body handle in Wheel builder").position().clone();
		// Build body & collider
		let body = RigidBodyBuilder::dynamic()
			//.position(add_isometries(&vehicle_position, &self.position_wrt_body))
			.ccd_enabled(true)
			.build();
		let body_handle = bodies.insert(body);
		let collider = static_.build_rapier_collider();
		let collider_handle = colliders.insert_with_parent(collider, body_handle, bodies);
		// Steering with joints: https://docs.rs/rapier3d/0.17.2/rapier3d/dynamics/struct.RevoluteJointBuilder.html
		#[cfg(feature = "backend")]
		let steering = {
			let (wheel_anchor_handle, wheel_position_wrt_anchor, steering): (RigidBodyHandle, Iso, WheelSteering) = match static_.steering {// The body that the wheel's "axle" will be attached to, either the vehicle itself or an intermediate body if it steers
				true => {
					// Build intermediate body
					let mut body = RigidBodyBuilder::dynamic()
						.position(add_isometries(&vehicle_position, &static_.position_wrt_body))
						.build();
					body.set_additional_mass_properties(
						MassProperties {
							local_com: point![0.0, 0.0, 0.0],
							inv_mass: 1.0,
							inv_principal_inertia_sqrt: point![1.0, 1.0, 1.0].coords,
							principal_inertia_local_frame: UnitQuaternion::new(V3::y())
						},
						true
					);// This is to prevent the wheel anchor having zero mass/moment of inertia which is interpreted as immovable, TODO
					let body_handle = bodies.insert(body);
					// Joint
					let joint = RevoluteJointBuilder::new(V3::y_axis())
						.local_anchor1(matrix_to_opoint(static_.position_wrt_body.translation.vector))
						.local_anchor2(point![0.0, 0.0, static_.steering_rotation_axis_offset])// Caster wheel?
						.build();
					let joint_handle = joints.insert(v_body_handle, body_handle, joint, true);
					(body_handle, Iso::identity(), Some((body_handle, joint_handle)))
				},
				false => (v_body_handle, static_.position_wrt_body, None)
			};
			let joint = RevoluteJointBuilder::new(V3::x_axis())
				.local_anchor1(matrix_to_opoint(wheel_position_wrt_anchor.translation.vector))
				.local_anchor2(point![0.0, 0.0, 0.0])
				.build();
			joints.insert(wheel_anchor_handle, body_handle, joint, true);
			steering
		};
		#[cfg(not(feature = "backend"))]
		let steering = None;
		// Done
		let out = Self {
			static_: static_.clone(),
			body_handle,
			collider_handle,
			steering
		};
		out.set_position_wrt_body(bodies, &vehicle_position);
		if static_.steering {
			out.update_steering(0.0, bodies, joints);
		}
		out
	}
	pub fn set_position_wrt_body(&self, bodies: &mut RigidBodySet, vehicle_position: &Iso) {
		Self::set_position_wrt_body_static(bodies.get_mut(self.body_handle).expect("Unable to get wheel body w/ handle"), &self.static_.position_wrt_body, vehicle_position);
	}
	pub fn set_position_wrt_body_static(body: &mut RigidBody, position_wrt_vehicle: &Iso, vehicle_position: &Iso) {
		body.set_position(add_isometries(vehicle_position, position_wrt_vehicle), true);
	}
	pub fn update_steering(&self, steering: Float, bodies: &mut RigidBodySet, joints: &mut ImpulseJointSet) {
		if self.static_.steering {
			//let (_, joint_handle) = self.steering.expect("steering is None, but static data says this wheel is steering");
			//let joint = joints.get_mut(joint_handle).expect("Joint set does not contain wheel steering joint");
			// TODO: get this to work
			//joint.data.set_motor(JointAxis::Y, steering, 6.28, 1000000.0, 1000000.0);// TODO: I just picked random values, fix
			// Temporary hack
			bodies.get_mut(self.body_handle).expect("Unable to get wheel body w/ handle").apply_torque_impulse(vector![0.0, steering as Float * 10.0, 0.0], true);
		}
	}
}

#[derive(Serialize, Deserialize, Clone)]
pub struct VehicleStatic {// This is loaded from the resources and is identified by a vehicle name
	pub name: String,
	pub type_: String,
	pub mass: Float,
	pub ctr_g_hight: Float,
	pub drag: V2,
	pub wheels: Vec<WheelStatic>
}

impl VehicleStatic {
    /*pub fn build(&self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, joints: &mut ImpulseJointSet, vehicle_position_opt: Option<&Iso>, paths: &PathSet) ->
	(
		RigidBodyHandle,
		ColliderHandle,
		Vec<Wheel>
	) {
		// Get position, otherwise use default
		let default_binding = Iso::identity();// Compiler said to do this, and it worked. Don't touch it.
		let vehicle_position = match vehicle_position_opt {
			Some(pos) => pos,
			None => &default_binding//Iso::new(UnitQuaternion::new(V3::x_axis()), V3::new(0., 0., 0.))// TODO: fix default value
		};
		// Body
        let mut body = match dynamic {
			true => RigidBodyBuilder::dynamic().ccd_enabled(true).build(),
			false => RigidBodyBuilder::kinematic_position_based().build()// The client does no actual simulation, so use this body type
		};
		body.set_position(*vehicle_position, true);
		let body_handle = bodies.insert(body);
		let collider = self.build_rapier_collider();
		let collider_handle = colliders.insert_with_parent(collider, body_handle, bodies);
		// Wheels
		let mut wheels = Vec::<Wheel>::new();
		for w_static in &self.wheels {
			wheels.push(Wheel::build(w_static, bodies, colliders, joints, body_handle));
			//joints.insert(body_handle, w_body_handle, joint, true);
		}
		// Set mass
		let b_ref = bodies.get_mut(body_handle).expect("Unable to get body w/ handle");
		b_ref.set_additional_mass(self.mass as Float - b_ref.mass(), true);
		//colliders.get_mut(c_handle).unwrap().
		(
			body_handle,
			collider_handle,
			wheels
		)
    }*/
	pub fn build_rapier_collider(&self) -> Collider {
		ColliderBuilder::cuboid(2.0, 0.5, 10.0)
			.restitution(0.2)// TODO
			.build()// Basic shape for now, TODO: change
	}
}

#[derive(Serialize, Deserialize)]
pub struct VehicleSave {// This is saved to and loaded from a world (game save file)
	pub static_name: String,// Name of static vehicle file
	body_state: BodyStateSerialize
}

#[derive(Serialize, Deserialize)]
pub struct VehicleSend {// This is sent over the network to the client
	static_name: String,// Name of static vehicle file
	pub latest_forces: Option<VehicleLinearForces>,
	pub body_state: BodyStateSerialize,
	input: Option<InputData>,
	latest_input_t: u64
}

impl VehicleSend {
	#[cfg(feature = "frontend")]
    pub fn update_bevy_camera_transform(&self, transform: &mut Transform) {
		let p: &Iso = &self.body_state.position;
        //*transform = Transform::from_xyz(50.0, 10.0, 50.0).looking_at(Vec3{x: v.0 as Float, y: v.1 as Float, z: v.2 as Float}, Vec3::Y);
        //*transform = transform.looking_at(p.translation.into(), Vec3::Y);
		let mut new_trans = Transform::default();
		// Create rotation quat
		let rot_offset = Quat::from_rotation_x(-0.2);
		let mut rot: Quat = p.rotation.into();
		rot = rot.mul_quat(rot_offset);
		new_trans.translation = p.translation.into();
		// 2nd POV
		let mut pov_trans = Transform::default();
		//pov_trans.translation = Vec3{x: 0.0, y: 0.0, z: -30.0};
		//pov_trans.rotation = rot;
		new_trans.translation = new_trans.translation + rot.mul_vec3(Vec3{x: 0.0, y: 0.0, z: 30.0});
		new_trans.rotation = rot;
		*transform = new_trans;
		/* *transform = Transform {
			translation: p.translation.into(),
    		rotation: p.rotation.into(),
    		scale: Vec3::ONE,
		}.looking_at();*/
    }
	pub fn update_rapier(&self, bodies: &mut RigidBodySet, wheels: Vec<&mut Wheel>) {
		self.check_wheel_positions(bodies, wheels);
	}
	pub fn check_wheel_positions(&self, bodies: &mut RigidBodySet, wheels: Vec<&mut Wheel>) {
		for w in wheels {
			w.set_position_wrt_body(bodies, &self.body_state.position);
		}
	}
}

#[cfg_attr(feature = "frontend", derive(Component))]
pub struct Vehicle {// This is used for physics
	pub static_: Arc<VehicleStatic>,
	latest_forces: Option<VehicleLinearForces>,
	latest_input: Option<InputData>,
	latest_input_t: u64,
	physics_controller: BodyPhysicsController
}

impl Vehicle {
	#[cfg(feature = "backend")]
	pub fn build(
		save: &VehicleSave,
		static_: Arc<VehicleStatic>,
		bodies: &mut RigidBodySet,
		colliders: &mut ColliderSet,
		joints: &mut ImpulseJointSet,
		paths: &PathSet
	) -> Result<Vehicle, Box<dyn Error>> {
		let physics_controller = BodyPhysicsController::build(&save.body_state, bodies, colliders, joints, paths, static_.clone());
		// Done
		Ok(Self {
			static_,
			latest_forces: None,
			latest_input: None,
			latest_input_t: 0u64,
			physics_controller
		})
	}
	pub fn update_user_input(&mut self, control: InputData) {
		// IMPORTANT: ONLY RUN WHEN THERE IS NEW USER DATA
		self.latest_input = Some(control);
		self.latest_input_t = get_unix_ts_secs_u64();
	}
	pub fn update_physics(&mut self, dt: Float, fluid_density: Float, bodies: &mut RigidBodySet, joints: &mut ImpulseJointSet, paths: &PathSet) {
		// Run every frame
		//update(&mut self, dt: Float, drag_force: V3, latest_input: Option<InputData>, paths: &PathSet, bodies: &mut RigidBodySet, joints: &mut ImpulseJointSet)
		let mut lin_forces = VehicleLinearForces::default();
		lin_forces.drag = self.drag_force(fluid_density, bodies, paths).z;
		self.physics_controller.update(dt, &mut lin_forces, &self.latest_input, paths, bodies, joints);
		// Save forces
		self.latest_forces = Some(lin_forces);
	}
	pub fn recover_from_flip(&self, physics_state: &mut PhysicsState) {
		self.physics_controller.recover_from_flip(physics_state);
	}
	#[cfg(feature = "backend")]
	pub fn create_serialize_state(&self, bodies: &RigidBodySet, paths: &PathSet) -> BodyStateSerialize {
		self.physics_controller.serializable(bodies, paths)
	}
	#[cfg(feature = "backend")]
	pub fn save(&self, body_set: &RigidBodySet, paths: &PathSet) -> VehicleSave {
		VehicleSave {
			static_name: self.static_.name.clone(),
			body_state: self.create_serialize_state(body_set, paths)
		}
	}
	#[cfg(feature = "backend")]
	pub fn send(&self, body_set: &RigidBodySet, paths: &PathSet) -> VehicleSend {
		VehicleSend {
			static_name: self.static_.name.clone(),
			latest_forces: self.latest_forces.clone(),
			body_state: self.create_serialize_state(body_set, paths),
			input: self.latest_input,
			latest_input_t: self.latest_input_t
		}
	}
	#[cfg(feature = "backend")]
	pub fn drag_force(&self, fluid_density: Float, bodies: &RigidBodySet, paths: &PathSet) -> V3 {
		// Drag force wrt this vehicle
		V3::zeros()// TODO
	}
}

#[derive(Serialize, Deserialize)]
pub struct BodyStateSerialize {// This is saved/sent over the network to the client. It will NOT be used inside the server game logic.
	pub position: Iso,
	pub lin_vel: V3,
	pub path_bound_opt: Option<PathBoundBodyState>
}

impl BodyStateSerialize {
	pub fn init_rapier_body(&self, body: &mut RigidBody) {
		body.set_position(self.position, true);// TODO: velocity
	}
	pub fn from_rapier_body(body: &RigidBody, path_bound_opt: Option<PathBoundBodyState>) -> Self {
		Self {
			position: *body.position(),
			lin_vel: *body.linvel(),
			path_bound_opt
		}
	}
}

pub enum BodyPhysicsController {
	RegularPhysics{
		body_handle: RigidBodyHandle,
		collider_handle: ColliderHandle,
		wheels: Vec<Wheel>
	},
	PathBound(PathBoundBodyState, Arc<VehicleStatic>)
}

impl BodyPhysicsController {
	pub fn build (
		body_state: &BodyStateSerialize,
		bodies: &mut RigidBodySet,
		colliders: &mut ColliderSet,
		joints: &mut ImpulseJointSet,
		paths: &PathSet,
		static_: Arc<VehicleStatic>
	) -> BodyPhysicsController {
		match &body_state.path_bound_opt {
			Some(path_bound_state) => {
				BodyPhysicsController::PathBound((*path_bound_state).clone(), static_.clone())
			},
			None => {
				// Body
				let mut body = RigidBodyBuilder::dynamic().ccd_enabled(true).build();
				body_state.init_rapier_body(&mut body);
				let body_handle = bodies.insert(body);
				let collider = static_.build_rapier_collider();
				let collider_handle = colliders.insert_with_parent(collider, body_handle, bodies);
				// Wheels
				let mut wheels = Vec::<Wheel>::new();
				for w_static in &static_.wheels {
					wheels.push(Wheel::build(w_static, bodies, colliders, joints, body_handle));
					//joints.insert(body_handle, w_body_handle, joint, true);
				}
				// Set mass
				let b_ref = bodies.get_mut(body_handle).expect("Unable to get body w/ handle");
				b_ref.set_additional_mass(static_.mass as Float - b_ref.mass(), true);
				// Done
				BodyPhysicsController::RegularPhysics{
					body_handle,
					collider_handle,
					wheels
				}
			}
		}
	}
	pub fn serializable(&self, bodies: &RigidBodySet, paths: &PathSet) -> BodyStateSerialize {
		match self {
			Self::RegularPhysics{body_handle, ..} => {
				let body: &RigidBody = bodies.get(*body_handle).expect("Unable to get body with BodyPhysicsController::RegularPhysics.body_handle");
				BodyStateSerialize::from_rapier_body(body, None)
			},
			Self::PathBound(path_body_state, ..) => {
				let path: &Path = paths.get_with_ref(&path_body_state.path_ref).expect("Unable to get path with path body state");
				path.create_body_state(path_body_state.clone())
			}
		}
	}
	pub fn update(&mut self, dt: Float, forces: &mut VehicleLinearForces, latest_input: &Option<InputData>, paths: &PathSet, bodies: &mut RigidBodySet, joints: &mut ImpulseJointSet) {
		match self {
			Self::RegularPhysics{body_handle, collider_handle, wheels} => match latest_input {
				Some(control) => match bodies.get_mut(*body_handle) {
					Some(body) => {
						// Get vehicle position and velocity
						let position = body.position().clone();
						// Forces
						body.reset_forces(true);
						body.reset_torques(true);
						body.add_force(position.rotation.transform_point(&P3::new(0.0, 0.0, -(control.speed as Float * 5.0))).coords, true);
						let torque_magnitude: Float = 800.0;
						body.add_torque(P3::new(0.0, control.steering as Float * torque_magnitude, 0.0).coords, true);
						/*for w in self.wheels.iter() {
							w.update_steering(control.steering as Float, bodies, joints);
						}*/
					},
					None => panic!("Unable to get vehicle body during update")
				},
				None => {}
			},
			Self::PathBound(ref mut path_body_state, v_static) => {
				// Get path
				let path: &Path = paths.get_with_ref(&path_body_state.path_ref).unwrap();
				// User input
				forces.drive = match latest_input {
					Some(input) => {
						// Power = force * velocity, force = power / velocity
						let vel_raw = path.get_real_velocity(path_body_state);
						let vel_corrected = if vel_raw >= 0.0 {// Make sure it cannot == 0
							vel_raw + 1e-3
						}
						else {
							vel_raw - 1e-3
						};
						let force_limit = v_static.mass * STATIONARY_DRIVE_ACC_LIMIT;// F = ma
						let force = (input.power as Float / vel_corrected).clamp(-force_limit, force_limit);
						// Done
						force
					},
					None => 0.0
				};
				// Update
				path.update_body(dt, forces, &v_static, path_body_state);
			}
		}
	}
	pub fn recover_from_flip(&self, physics_state: &mut PhysicsState) {
		match self {
			Self::RegularPhysics{body_handle, collider_handle, wheels} => {
				// TODO: fix
				let body = physics_state.bodies.get_mut(*body_handle).expect("Unable to get vehicle body during unflip");
				body.set_rotation(UnitQuaternion::identity(), true);// TODO: clearance above ground
				body.set_translation(body.translation() + vector![0.0, 10.0, 0.0], true);
				physics_state.bodies.propagate_modified_body_positions_to_colliders(&mut physics_state.colliders);
			},
			Self::PathBound(..) => {}
		}
	}
	pub fn is_path_bound(&self) -> bool {
		match self {
			Self::RegularPhysics{..} => false,
			Self::PathBound(..) => true
		}
	}
}

#[derive(Serialize, Deserialize, Default, Clone)]
pub struct VehicleLinearForces {// Standard units, so Newtons
	pub rolling_resistance: Float,
	pub drag: Float,
	pub drive: Float,
	pub gravity: Float,
	pub other: Float
}

impl VehicleLinearForces {
	pub fn sum(&self) -> Float {
		self.rolling_resistance + self.drag + self.drive + self.gravity + self.other
	}
}