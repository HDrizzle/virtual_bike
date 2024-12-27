//! Defines a vehicle in the game
//! Potentially usefull https://www.sheldonbrown.com/rinard/aero/formulas.html
//! Any vehicle can be in one of two states: normal physics and path-bound. Normal physics means it will simply be controlled by the physics engine (Rapier 3D).
//! when path-bound it will be controlled by the specific Path it is bound to. it's velocity will be a single scalar value whose sign indicates which direction it is travelling on the path wrt itself.

use std::{error::Error, rc::Rc, ops::AddAssign};
#[cfg(feature = "client")]
use std::{io::Write, net::SocketAddr};
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
#[cfg(feature = "client")]
use bevy::prelude::*;
#[cfg(feature = "client")]
use bevy_inspector_egui::bevy_egui::egui::Ui;// Importing from re-export to prevent conflicting versions of bevy_egui

use crate::{prelude::*, map::path::PathBoundBodyState};
#[cfg(feature = "server")]
use crate::physics::PhysicsController;

const STATIONARY_DRIVE_ACC_LIMIT: Float = 10.0;// m/s^2

#[derive(Serialize, Deserialize, Clone)]
pub struct WheelStatic {
	pub dia: Float,
	pub width: Float,
	pub friction: Float,
	/// Relative to the vehicle body
	pub position_wrt_body: Iso,
	pub driven: bool,
	pub steering: bool,
	/// A non-zero value of this will result in something similar to a shopping-cart wheel where the wheel axis does not align with the turning axis
	pub steering_rotation_axis_offset: Float,
}

#[derive(Clone)]
#[cfg_attr(feature = "client", derive(Component))]
#[allow(unused)]
pub struct Wheel {
	static_: WheelStatic,
	angle: Float
}

/// This is loaded from the resources and is identified by a vehicle name
#[derive(Serialize, Deserialize, Clone)]
pub struct VehicleStatic {
	/// Type of vehicle this represents
	pub type_name: String,
	pub mass: Float,
	pub ctr_g_hight: Float,
	pub drag: Float,
	/// For calculating drag
	pub cross_sectional_area: Float,
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
	/// Copied from default trait implementation to add "#Scene0" to asset path
	fn load_to_bevy(name: &str, server_addr: SocketAddr, asset_server: &AssetServer) -> Result<Handle<Self::BevyAssetType>, String> {
		let path_raw = Self::get_path(name, server_addr);
		match path_raw.strip_prefix("assets/") {
			Some(path) => Ok(asset_server.load(path.to_owned() + "#Scene0")),
			None => Err(format!("Unable to strip \"assets/\" off the beginning of \"{}\" so it can be compatible with bevy's asset server", &path_raw))
		}
	}
}

/// This is saved to and loaded from a world (game save file)
#[derive(Serialize, Deserialize)]
pub struct VehicleSave {
	/// Name of static vehicle file
	pub type_: String,
	pub body_state: BodyStateSerialize
}

/// This is sent over the network to the client
#[derive(Serialize, Deserialize)]
pub struct VehicleSend {
	/// Name of static vehicle file
	pub type_: String,
	pub latest_forces: Option<BodyForces>,
	/// Description of forces acting on vehicle when on path
	pub path_forces_opt: Option<PathBodyForceDescription>,
	pub route_opt: Option<GenericRef<Route>>,
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

/// This is used for physics
#[cfg(feature = "server")]
pub struct Vehicle {
	pub static_: Rc<VehicleStatic>,
	pub latest_forces: Option<BodyForces>,
	pub path_forces_opt: Option<PathBodyForceDescription>,
	pub latest_input: Option<InputData>,
	pub latest_input_t: u64,
	pub physics_controller: Box<dyn PhysicsController>
}

#[cfg(feature = "server")]
impl Vehicle {
	pub fn build(
		save: &VehicleSave,
		static_: Rc<VehicleStatic>
	) -> Result<Vehicle, Box<dyn Error>> {
		let physics_controller = save.body_state.build_physics_controller(static_.clone());
		// Done
		Ok(Self {
			static_,
			latest_forces: None,
			path_forces_opt: None,
			latest_input: None,
			latest_input_t: 0u64,
			physics_controller
		})
	}
	/// IMPORTANT: ONLY RUN WHEN THERE IS NEW USER INPUT
	pub fn update_user_input(&mut self, control: InputData) {
		self.latest_input = Some(control);
		self.latest_input_t = get_unix_ts_secs_u64();
	}
	/// Must be run every frame
	pub fn update_physics(&mut self, dt: Float, fluid_density: Float, path_set: &PathSet, gravity: Float) {
		//update(&mut self, dt: Float, drag_force: V3, latest_input: Option<InputData>, paths: &PathSet, bodies: &mut RigidBodySet, joints: &mut ImpulseJointSet)
		let (forces, path_forces_opt): (BodyForces, Option<PathBodyForceDescription>) = self.physics_controller.update(PhysicsUpdateArgs{
			dt,
			gravity,
			path_set,
			latest_input: &self.latest_input,
			drag_force_calculator: &|lin: V3, _ang: V3| -> BodyForces {BodyForces::new(V3::new(0.0, 0.0, crate::physics::fluid_drag_force(fluid_density, lin.z, self.static_.drag, self.static_.cross_sectional_area)), V3::zeros())}
		});
		// Save forces
		self.latest_forces = Some(forces);
		self.path_forces_opt = path_forces_opt;
	}
	pub fn create_serialize_state(&self, paths: &PathSet) -> BodyStateSerialize {
		self.physics_controller.serializable(paths)
	}
	pub fn save(&self, paths: &PathSet) -> VehicleSave {
		VehicleSave {
			type_: self.static_.type_name.clone(),
			body_state: self.create_serialize_state(paths)
		}
	}
	pub fn send(&self, paths: &PathSet) -> VehicleSend {
		VehicleSend {
			type_: self.static_.type_name.clone(),
			latest_forces: self.latest_forces.clone(),
			path_forces_opt: self.path_forces_opt.clone(),
			route_opt: self.physics_controller.get_route_opt(&paths.generic.routes),
			body_state: self.create_serialize_state(paths),
			input: self.latest_input,
			latest_input_t: self.latest_input_t
		}
	}
	/*pub fn calculate_forces(&self, lin: V3, ang: V3, fluid_density: Float) -> BodyForces {
		// Drag force wrt this vehicle
		BodyForces::default()
	}*/
}

/// This is saved/sent over the network to the client. It will NOT be used inside the server game logic.
#[derive(Serialize, Deserialize, Clone)]
pub struct BodyStateSerialize {
	pub position: Iso,
	pub lin_vel: V3,
	pub ang_vel: V3,
	pub path_bound_opt: Option<PathBoundBodyState>
}

impl BodyStateSerialize {
	#[cfg(feature = "server")]
	pub fn build_physics_controller(
		&self,
		v_static: Rc<VehicleStatic>
	) -> Box<dyn PhysicsController> {
		Box::new(VehiclePathBoundController::build(self.path_bound_opt.clone().expect("Currently, path bound is the only option for vehicle physics"), v_static))
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
	fn serializable(&self, path_set: &PathSet) -> BodyStateSerialize {
		let path: &Path = path_set.paths.get_item_tuple(&self.state.path_query).expect(&format!("Unable to get path with body state path query `{:?}`", &self.state.path_query)).1;
		path.generic.create_body_state(self.state.clone())
	}
	fn update(&mut self, args: PhysicsUpdateArgs) -> (BodyForces, Option<PathBodyForceDescription>) {
		// Forces
		let mut forces = PathBodyForceDescription::default();
		// Get path
		let path: &Path = args.path_set.paths.get_item_tuple(&self.state.path_query).expect("Unable to get path with path body state").1;
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
		// old
		//forces.lin += V3::new(0.0, 0.0, drive_force + brake_force + (path.generic.sideways_gravity_force_component(&self.state.pos, &*self.v_static, args.gravity) * bool_sign(self.state.forward) as Float));
		forces.drive = drive_force;
		forces.brake = brake_force;
		forces.drag = (args.drag_force_calculator)(V3::new(0.0, 0.0, self.state.velocity), V3::zeros()).lin.z;
		forces.gravity = path.generic.sideways_gravity_force_component(&self.state.pos, &*self.v_static, args.gravity) * (bool_sign(self.state.forward) as Float);
		forces.rolling_resistance = 0.0;// TODO
		// Update
		self.state.update(args.dt, &forces, &self.v_static, &args.path_set);
		//path.update_body(args.dt, &forces, &self.v_static, &mut self.state);
		(BodyForces::from_path_bound_forces(&forces), Some(forces))
	}
	fn get_route_opt(&self, routes: &GenericDataset<Route>) -> Option<GenericRef<Route>> {
		match &self.state.route_query_opt {
			Some(route_query) => Some(routes.get_item_tuple(route_query).expect(&format!("Route query ({:?}) invalid", route_query)).0.clone()),
			None => None
		}
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

/// Standard units, so Newtons. Always wrt the body even if it is facing backwards on a path
#[derive(Serialize, Deserialize, Default, Clone)]
pub struct BodyForces {
	pub lin: V3,
	pub ang: V3
}

impl BodyForces {
	pub fn new(lin: V3, ang: V3) -> Self {
		Self {
			lin,
			ang
		}
	}
	/// Creates a new Self from the forces acting on a path bound object. These forces are considered to act along the local Z-axis of the object.
	pub fn from_path_bound_forces(path_forces: &PathBodyForceDescription) -> Self {
		Self {
			lin: V3::new(0.0, 0.0, path_forces.sum()),
			ang: V3::zeros()
		}
	}
}

impl AddAssign for BodyForces {
	fn add_assign(&mut self, rhs: Self) {
		// Linear
		self.lin += rhs.lin;
		// Angular
		self.ang += rhs.ang;
	}
}

/// Describes all forces acting to acc/decelerate a path bound body, sent to client
/// All forces are Newtons
#[derive(Serialize, Deserialize, Default, Clone)]
pub struct PathBodyForceDescription {
	/// Drive force, power / speed
	pub drive: Float,
	/// Brake force
	pub brake: Float,
	/// This one can really suck
	pub drag: Float,
	/// Ugghhh hills
	pub gravity: Float,
	/// Only important on like gravel
	pub rolling_resistance: Float,
	/// In case there's something I haven't thought of, (force, description)
	pub other_opt: Option<(Float, String)>
}

impl PathBodyForceDescription {
	/// Sum, including other
	pub fn sum(&self) -> Float {
		let almost = self.drive + self.brake + self.drag + self.gravity + self.rolling_resistance;
		match self.other_opt {
			Some((other_force, _)) => almost + other_force,
			None => almost
		}
	}
	/// Renders to a part of an egui
	#[cfg(feature = "client")]
	pub fn render_to_egui(&self, ui: &mut Ui) {
		ui.label("Forces");
		ui.label(format!("Drive: {:.2}", self.drive));
		ui.label(format!("Brake: {:.2}", self.brake));
		ui.label(format!("Drag: {:.2}", self.drag));
		ui.label(format!("Gravity: {:.2}", self.gravity));
		ui.label(format!("Rolling resistance: {:.2}", self.rolling_resistance));
		match &self.other_opt {
			Some((other_force, desc)) => {
				ui.label(format!("Other ({}): {:.2}", desc, other_force));
			}
			None => {}
		}
		ui.label(format!("Total: {:.2}", self.sum()));
	}
}