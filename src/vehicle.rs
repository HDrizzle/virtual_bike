//! Defines a vehicle in the game
//! Potentially usefull https://www.sheldonbrown.com/rinard/aero/formulas.html
//! Any vehicle can be in one of two states: normal physics and path-bound. Normal physics means it will simply be controlled by the physics engine (Rapier 3D).
//! when path-bound it will be controlled by the specific Path it is bound to. it's velocity will be a single scalar value whose sign indicates which direction it is travelling on the path wrt itself.

use std::{error::Error, rc::Rc, ops::AddAssign, io::Write, net::SocketAddr};
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
#[cfg(feature = "client")]
use bevy::{prelude::*, ecs::component::Component};
use nalgebra::{vector, point, UnitQuaternion};
#[cfg(feature = "server")]
use easy_gltf;

// Rapier 3D physics
#[cfg(any(feature = "server", feature = "debug_render_physics"))]
use rapier3d::{prelude::*, math::Real};
//use nalgebra::{Isometry3, Vector3, vector, Point3, point, geometry::Quaternion, UnitQuaternion, Const};

use crate::{prelude::*, map::path::PathBoundBodyState};
#[cfg(feature = "server")]
use crate::physics::PhysicsController;

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
	#[cfg(any(feature = "server", feature = "debug_render_physics"))]
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

#[cfg(feature = "server")]
type WheelSteering = Option<(RigidBodyHandle, ImpulseJointHandle)>;

#[derive(Clone)]
#[cfg_attr(feature = "frontend", derive(Component))]
pub struct Wheel {
	static_: WheelStatic,
	#[cfg(feature = "server")] body_handle: RigidBodyHandle,
	#[cfg(feature = "server")] collider_handle: ColliderHandle,
	#[cfg(feature = "server")] steering: WheelSteering// None if not static_.steering or built in client
}

#[cfg(feature = "server")]
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
		#[cfg(feature = "server")]
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
		#[cfg(feature = "client")]
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
	/// Type of vehicle this represents
	pub type_name: String,
	pub mass: Float,
	pub ctr_g_hight: Float,
	pub drag: V2,
	pub wheels: Vec<WheelStatic>
}

#[derive(Serialize, Deserialize)]
pub struct VehicleStaticModel {
	static_name: String,
	glb_data: Vec<u8>
}

impl VehicleStaticModel {
	pub fn new(
		static_name: String,
		glb_data: Vec<u8>
	) -> Self {
		Self {
			static_name,
			glb_data
		}
	}
}

#[cfg(feature = "client")]
impl CacheableBevyAsset for VehicleStaticModel {
	const CACHE_SUB_DIR: &'static str = "static_vehicles/";
	type BevyAssetType = Scene;
	fn name(&self) -> String {
		self.static_name.clone()
	}
	fn cache_path(name: &str) -> String {
		format!("{}/model.glb", name)
	}
	fn write_to_file(&self, file: &mut std::fs::File) -> Result<(), String> {
		to_string_err(file.write_all(&self.glb_data))
	}
	// Copied from default trait implementation to add "#Scene0" to asset path
	fn load_to_bevy(name: &str, server_addr: SocketAddr, asset_server: &AssetServer) -> Result<Handle<Self::BevyAssetType>, String> {
		let path_raw = Self::get_path(name, server_addr);
		match path_raw.strip_prefix("assets/") {
			Some(path) => Ok(asset_server.load(path.to_owned() + "#Scene0")),
			None => Err(format!("Unable to strip \"assets/\" off the beginning of \"{}\" so it can be compatible with bevy's asset server", &path_raw))
		}
	}
}

#[cfg(feature = "server")]
impl VehicleStatic {
	pub fn build_rapier_collider(&self) -> Collider {
		let scenes = easy_gltf::load(&(resource_interface::VEHICLES_DIR.to_owned() + &self.type_name + "/model.glb")).expect("Unable to load gltf for static vehicle model file");
		for scene in scenes {
			/*println!(
				"Cameras: #{}  Lights: #{}  Models: #{}",
				scene.cameras.len(),
				scene.lights.len(),
				scene.models.len()
			)*/
			for model in scene.models {// https://docs.rs/easy-gltf/latest/easy_gltf/model/struct.Model.html#
				// Extract vertices
				let mut vertices = Vec::<P3>::new();
				for gltf_vertex in model.vertices() {
					let p = gltf_vertex.position;
					vertices.push(P3::new(p.x, p.y, p.z));
				}
				// Extract indices
				let mut indices = Vec::<[u32; 3]>::new();
				let mut curr_triangle: [u32; 3] = [0; 3];// Placeholder
				let mut i = 0;
				for flat_index in model.indices().expect(&format!("Model for static vehicle {} has no indices", &self.type_name)) {
					curr_triangle[i] = *flat_index;
					i += 1;
					if i == 3 {
						indices.push(curr_triangle);
						curr_triangle = [0; 3];
						i = 0;
					}
				}
				// https://docs.rs/rapier3d/latest/rapier3d/geometry/struct.ColliderBuilder.html#method.trimesh
				return ColliderBuilder::convex_decomposition(&vertices, &indices).restitution(0.2).mass(self.mass).friction(0.2).build();
			}
		}
		panic!("no scenes or nodes with primitives found in model file for vehicle type {}", &self.type_name);
	}
}

#[derive(Serialize, Deserialize)]
pub struct VehicleSave {// This is saved to and loaded from a world (game save file)
	pub type_: String,// Name of static vehicle file
	pub body_state: BodyStateSerialize
}

#[derive(Serialize, Deserialize)]
pub struct VehicleSend {// This is sent over the network to the client
	pub type_: String,// Name of static vehicle file
	pub latest_forces: Option<BodyForces>,
	pub body_state: BodyStateSerialize,
	input: Option<InputData>,
	latest_input_t: u64
}

impl VehicleSend {
	#[cfg(feature = "client")]
    pub fn update_bevy_camera_transform(&self, transform: &mut Transform) {
		let p: &Iso = &self.body_state.position;
        //*transform = Transform::from_xyz(50.0, 10.0, 50.0).looking_at(Vec3{x: v.0 as Float, y: v.1 as Float, z: v.2 as Float}, Vec3::Y);
        //*transform = transform.looking_at(p.translation.into(), Vec3::Y);
		let mut new_trans = Transform::default();
		// Create rotation quat
		let rot_offset = Quat::from_rotation_x(-0.2);
		let mut rot: Quat = nalgebra_quat_to_bevy_quat(&p.rotation);
		rot = rot.mul_quat(rot_offset);
		new_trans.translation = nalgebra_vec3_to_bevy_vec3(&p.translation.vector);
		// 2nd POV
		new_trans.translation = new_trans.translation + rot.mul_vec3(Vec3{x: 0.0, y: 0.0, z: 30.0});
		new_trans.rotation = rot;
		*transform = new_trans;
		/* *transform = Transform {
			translation: p.translation.into(),
    		rotation: p.rotation.into(),
    		scale: Vec3::ONE,
		}.looking_at();*/
    }
	/*pub fn update_rapier(&self, bodies: &mut RigidBodySet, wheels: Vec<&mut Wheel>) {
		self.check_wheel_positions(bodies, wheels);
	}
	pub fn check_wheel_positions(&self, bodies: &mut RigidBodySet, wheels: Vec<&mut Wheel>) {
		for w in wheels {
			w.set_position_wrt_body(bodies, &self.body_state.position);
		}
	}*/
}

#[cfg(feature = "server")]//#[cfg_attr(feature = "frontend", derive(Component))]
pub struct Vehicle {// This is used for physics
	pub static_: Rc<VehicleStatic>,
	pub latest_forces: Option<BodyForces>,
	pub latest_input: Option<InputData>,
	pub latest_input_t: u64,
	pub physics_controller: Box<dyn PhysicsController>
}

#[cfg(feature = "server")]
impl Vehicle {
	pub fn build(
		save: &VehicleSave,
		static_: Rc<VehicleStatic>,
		bodies: &mut RigidBodySet,
		colliders: &mut ColliderSet,
		joints: &mut ImpulseJointSet,
		paths: &PathSet
	) -> Result<Vehicle, Box<dyn Error>> {
		let physics_controller = save.body_state.build_physics_controller(bodies, colliders, joints, paths, static_.clone());
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
	pub fn update_physics(&mut self, dt: Float, fluid_density: Float, physics_state: &mut PhysicsState, paths: &PathSet, gravity: Float) {
		// Run every frame
		//update(&mut self, dt: Float, drag_force: V3, latest_input: Option<InputData>, paths: &PathSet, bodies: &mut RigidBodySet, joints: &mut ImpulseJointSet)
		let forces = self.physics_controller.update(PhysicsUpdateArgs{
			dt,
			gravity,
			rapier: physics_state,
			paths,
			latest_input: &self.latest_input,
			extra_forces_calculator: &|lin: V3, ang: V3| -> BodyForces {BodyForces::default()}// TODO
		});
		// Save forces
		self.latest_forces = Some(forces);
	}
	pub fn recover_from_flip(&mut self, physics_state: &mut PhysicsState) {
		self.physics_controller.recover_from_flip(physics_state);
	}
	pub fn create_serialize_state(&self, bodies: &RigidBodySet, paths: &PathSet) -> BodyStateSerialize {
		self.physics_controller.serializable(bodies, paths)
	}
	pub fn save(&self, body_set: &RigidBodySet, paths: &PathSet) -> VehicleSave {
		VehicleSave {
			type_: self.static_.type_name.clone(),
			body_state: self.create_serialize_state(body_set, paths)
		}
	}
	pub fn send(&self, body_set: &RigidBodySet, paths: &PathSet) -> VehicleSend {
		VehicleSend {
			type_: self.static_.type_name.clone(),
			latest_forces: self.latest_forces.clone(),
			body_state: self.create_serialize_state(body_set, paths),
			input: self.latest_input,
			latest_input_t: self.latest_input_t
		}
	}
	pub fn calculate_forces(&self, lin: V3, ang: V3, fluid_density: Float) -> BodyForces {
		// Drag force wrt this vehicle
		BodyForces::default()
	}
}

#[derive(Serialize, Deserialize, Clone)]
pub struct BodyStateSerialize {// This is saved/sent over the network to the client. It will NOT be used inside the server game logic.
	pub position: Iso,
	pub lin_vel: V3,
	pub ang_vel: V3,
	pub path_bound_opt: Option<PathBoundBodyState>
}

impl BodyStateSerialize {
	#[cfg(any(feature = "server", feature = "debug_render_physics"))]
	pub fn init_rapier_body(&self, body: &mut RigidBody) {
		body.set_position(self.position, true);
		body.set_linvel(self.lin_vel, true);
		body.set_angvel(self.ang_vel, true);
	}
	#[cfg(feature = "server")]
	pub fn from_rapier_body(body: &RigidBody, path_bound_opt: Option<PathBoundBodyState>) -> Self {
		Self {
			position: *body.position(),
			lin_vel: *body.linvel(),
			ang_vel: *body.angvel(),
			path_bound_opt
		}
	}
	#[cfg(feature = "server")]
	pub fn build_physics_controller(
		&self,
		bodies: &mut RigidBodySet,
		colliders: &mut ColliderSet,
		joints: &mut ImpulseJointSet,
		paths: &PathSet,
		v_static: Rc<VehicleStatic>
	) -> Box<dyn PhysicsController> {
		match &self.path_bound_opt {
			Some(path_state) => {
				Box::new(VehiclePathBoundController::build(path_state.clone(), v_static))
			},
			None => {
				Box::new(VehicleRapierController::build(&self, bodies, colliders, joints, paths, v_static))
			}
		}
	}
}

#[cfg(feature = "server")]
pub struct VehicleRapierController {
	body_handle: RigidBodyHandle,
	collider_handle: ColliderHandle,
	wheels: Vec<Wheel>,
	v_static: Rc<VehicleStatic>
}

#[cfg(feature = "server")]
impl VehicleRapierController {
	pub fn build (
		body_state: &BodyStateSerialize,
		bodies: &mut RigidBodySet,
		colliders: &mut ColliderSet,
		joints: &mut ImpulseJointSet,
		paths: &PathSet,
		static_: Rc<VehicleStatic>
	) -> Self {
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
		Self {
			body_handle,
			collider_handle,
			wheels,
			v_static: static_.clone()
		}
	}
}

#[cfg(feature = "server")]
impl PhysicsController for VehicleRapierController {
	fn serializable(&self, bodies: &RigidBodySet, paths: &PathSet) -> BodyStateSerialize {
		let body: &RigidBody = bodies.get(self.body_handle).expect("Unable to get body with BodyPhysicsController::RegularPhysics.body_handle");
		BodyStateSerialize::from_rapier_body(body, None)
	}
	fn update(&mut self, args: PhysicsUpdateArgs) -> BodyForces {
		match args.latest_input {
			Some(control) => match args.rapier.bodies.get_mut(self.body_handle) {
				Some(body) => {
					// Get vehicle position and velocity
					let position = body.position().clone();
					// Forces
					body.reset_forces(true);
					body.reset_torques(true);
					body.add_force(position.rotation.transform_point(&P3::new(0.0, 0.0, -control.power)).coords, true);// Drive, TODO: fix
					body.add_force(position.rotation.transform_point(&P3::new(0.0, 0.0, control.brake * self.v_static.mass)).coords, true);// Brake
					let torque_magnitude: Float = 1000.0;
					body.add_torque(P3::new(0.0, control.steering as Float * torque_magnitude, 0.0).coords, true);
					/*for w in self.wheels.iter() {
						w.update_steering(control.steering as Float, bodies, joints);
					}*/
				},
				None => panic!("Unable to get vehicle body during update")
			},
			None => {}
		}
		BodyForces::default()
	}
	fn recover_from_flip(&mut self, physics_state: &mut PhysicsState) {
		// TODO: fix
		let body = physics_state.bodies.get_mut(self.body_handle).expect("Unable to get vehicle body during unflip");
		body.set_rotation(UnitQuaternion::identity(), true);
		body.set_translation(body.translation() + vector![0.0, 10.0, 0.0], true);// TODO: fix: arbitrary value
		physics_state.bodies.propagate_modified_body_positions_to_colliders(&mut physics_state.colliders);
	}
}

#[cfg(feature = "server")]
pub struct VehiclePathBoundController {
	state: PathBoundBodyState,
	v_static: Rc<VehicleStatic>
}

#[cfg(feature = "server")]
impl VehiclePathBoundController {
	pub fn build (
		state: PathBoundBodyState,
		v_static: Rc<VehicleStatic>
	) -> Self {
		Self {
			state,
			v_static: v_static.clone()
		}
	}
}

#[cfg(feature = "server")]
impl PhysicsController for VehiclePathBoundController{
	fn serializable(&self, _: &RigidBodySet, paths: &PathSet) -> BodyStateSerialize {
		let path: &Path = paths.get_path_with_ref(&self.state.path_ref).expect("Unable to get path with path body state");
		path.generic.create_body_state(self.state.clone())
	}
	fn update(&mut self, args: PhysicsUpdateArgs) -> BodyForces {
		// Forces
		let mut forces = BodyForces::default();
		forces += (args.extra_forces_calculator)(V3::new(0.0, 0.0, self.state.velocity), V3::zeros());// TODO
		// Get path
		let path: &Path = args.paths.get_path_with_ref(&self.state.path_ref).unwrap();
		// User input
		let (drive_force, brake_force): (Float, Float) = match args.latest_input {
			Some(input) => {
				// Power = force * velocity; force = power / velocity
				let vel_raw = self.state.velocity;
				let vel_corrected = if vel_raw >= 0.0 {// Make sure it cannot == 0
					vel_raw + 1e-3
				}
				else {
					vel_raw - 1e-3
				};
				let force_limit = self.v_static.mass * STATIONARY_DRIVE_ACC_LIMIT;// F = ma
				let drive_force = (input.power as Float / vel_corrected).clamp(-force_limit, force_limit);
				// Brake
				let brake_force: Float = input.brake * self.v_static.mass * -self.state.velocity;
				// Done
				(drive_force, brake_force)
			},
			None => (0.0, 0.0)
		};
		forces.lin += V3::new(0.0, 0.0, drive_force + brake_force + (path.generic.sideways_gravity_force_component(&self.state.pos, &*self.v_static, args.gravity) * bool_sign(self.state.forward) as Float));
		// Update
		self.state.update(args.dt, &forces, &self.v_static, &args.paths);
		//path.update_body(args.dt, &forces, &self.v_static, &mut self.state);
		forces
	}
}

/*#[derive(Serialize, Deserialize, Default, Clone)]
pub struct BodyLinearForces {// Standard units, so Newtons. Always wrt the body even if it is facing backwards on a path
	pub rolling_resistance: Float,
	pub drag: Float,
	pub drive: Float,
	pub gravity: Float,
	pub brake: Float,
	pub other: Float
}

impl BodyLinearForces {
	pub fn sum(&self) -> Float {
		self.rolling_resistance + self.drag + self.drive + self.gravity + self.brake + self.other
	}
}*/

#[derive(Serialize, Deserialize, Default, Clone)]
pub struct BodyForces {// Standard units, so Newtons. Always wrt the body even if it is facing backwards on a path
	pub lin: V3,
	pub ang: V3
}

impl AddAssign for BodyForces {
	fn add_assign(&mut self, rhs: Self) {
		// Linear
		self.lin += rhs.lin;
		// Angular
		self.ang += rhs.ang;
	}
}