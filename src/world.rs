//! World simulation

use std::{thread, sync::mpsc, rc::Rc, collections::HashMap, time::{Duration, Instant}};
#[cfg(all(test, feature = "server"))]
use std::sync::Arc;
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
#[cfg(feature = "client")]
#[cfg(feature = "debug_render_physics")]
use bevy_rapier3d::plugin::RapierContext;
#[cfg(feature = "client")]
use bevy::ecs::system::Resource;

// Rapier 3D physics
#[cfg(any(feature = "server", feature = "debug_render_physics"))]
use rapier3d::prelude::*;

use crate::prelude::*;
#[cfg(feature = "server")]
use crate::resource_interface::*;
/*#[cfg(feature = "client")]
#[cfg(feature = "debug_render_physics")]
use crate::client::play::VehicleBodyHandles;*/

#[cfg(feature = "server")]
use nalgebra::vector;

// Structs

/// Sent to every client when it first connects to the server, this contains data that does not change on the server's end througout a simulation
#[derive(Serialize, Deserialize, Clone)]
#[cfg_attr(feature = "client", derive(Resource))]
pub struct StaticData {
	pub map: SendMap,
	pub static_vehicles: HashMap<String, VehicleStatic>,// Vehicle type: VehicleStatic
	#[cfg(feature = "debug_render_physics")]
	pub partial_physics: PhysicsStateSend
}

impl StaticData {
	#[cfg(feature = "client")]
	#[cfg(feature = "debug_render_physics")]
	pub fn init_bevy_rapier_context(&mut self, context: &mut RapierContext) {
		// Debug feature
		self.partial_physics.init_bevy_rapier_context(context);// very important that this is the first thing that acts on the rapier context, NOT the map
		// Map
		self.map.generic.init_rapier(&mut context.bodies);
	}
	/*#[cfg(feature = "client")]
	// Probably outdated
	pub fn vehicle_models_to_load(&self, server_addr: IpAddr) -> Vec<String> {
		let mut out: Vec<String> = Vec::<String>::new();
		for (_, v) in self.static_vehicles.iter() {
			if !VehicleStaticModel::new(v.name.clone(), Vec::new()).is_already_cached(server_addr) {
				out.push(v.name.clone());
			}
		}
		// Done
		out
	}*/
	/// Compiles Vec of all vehicle types (static vehicle type names) being used
	#[cfg(feature = "client")]
	pub fn all_vehicle_types(&self) -> Vec<String> {
		let mut out: Vec<String> = Vec::<String>::new();
		for (_, v) in self.static_vehicles.iter() {
			if !out.contains(&v.type_name) {
				out.push(v.type_name.clone());
			}
		}
		// Done
		out
	}
	pub fn debug_print_sizes(&self) {
		println!("StaticData.debug_print_sizes() is running");
		{
			let map = bincode::serialize(&self.map).unwrap();
			println!("map: {}", map.len());
		}
		{
			let vehicles = bincode::serialize(&self.static_vehicles).unwrap();
			println!("vehicles: {}", vehicles.len());
		}
		#[cfg(feature = "debug_render_physics")]
		{
			let partial_physics = bincode::serialize(&self.partial_physics).unwrap();
			println!("partial_physics: {}", partial_physics.len());
		}
	}
}

/// Complete physics state for use by Rapier
/// Copied from https://rapier.rs/docs/user_guides/rust/serialization
#[cfg(feature = "server")]
pub struct PhysicsState {
	pub pipeline: PhysicsPipeline,
	pub islands: IslandManager,
	pub broad_phase: BroadPhase,
	pub narrow_phase: NarrowPhase,
	pub bodies: RigidBodySet,
	pub colliders: ColliderSet,
	pub impulse_joints: ImpulseJointSet,
	pub multibody_joints: MultibodyJointSet,
	pub ccd_solver: CCDSolver,
	pub query_pipeline: QueryPipeline,
	pub integration_parameters: IntegrationParameters,
	pub gravity: Vector<f32>,
	pub physics_hooks: (),
	pub event_handler: (),
}

#[cfg(feature = "server")]
impl PhysicsState {
	pub fn new(gravity: Float) -> Self {
		Self {
			pipeline: PhysicsPipeline::new(),
			islands: IslandManager::new(),
			broad_phase: BroadPhase::new(),
			narrow_phase: NarrowPhase::new(),
			bodies: RigidBodySet::new(),
			colliders: ColliderSet::new(),
			impulse_joints: ImpulseJointSet::new(),
			multibody_joints: MultibodyJointSet::new(),
			ccd_solver: CCDSolver::new(),
			query_pipeline: QueryPipeline::new(),
			integration_parameters: IntegrationParameters::default(),
			gravity: vector![0.0, gravity, 0.0],
			physics_hooks: (),
			event_handler: ()
		}
	}
	pub fn step(&mut self) {
		self.pipeline.step(
			&self.gravity,
			&self.integration_parameters,
			&mut self.islands,
			&mut self.broad_phase,
			&mut self.narrow_phase,
			&mut self.bodies,
			&mut self.colliders,
			&mut self.impulse_joints,
			&mut self.multibody_joints,
			&mut self.ccd_solver,
			None,
			&self.physics_hooks,
			&self.event_handler
		)
	}
	pub fn insert_body(&mut self, body: RigidBody) -> RigidBodyHandle {
		self.bodies.insert(body)
	}
	pub fn insert_collider_with_parent(&mut self, collider: Collider, body_handle: RigidBodyHandle) -> ColliderHandle {
		self.colliders.insert_with_parent(collider, body_handle, &mut self.bodies)
	}
	#[cfg(feature = "debug_render_physics")]
	#[cfg(feature = "server")]
	pub fn send(&self/*, vehicles: &HashMap<String, Vehicle>*/) -> PhysicsStateSend {// TODO: only send the states of vehicles, wheel mounts and wheels
		//let (mut bodies_v, mut colliders_v, mut impulse_joints_v): (Vec<RigidBodyHandle>, Vec<ColliderHandle>, Vec<ImpulseJointHandle>) = (Vec::new(), Vec::new(), Vec::new());
		PhysicsStateSend {
			islands: IslandManager::new(),//self.islands.clone(),
			bodies: self.bodies.clone(),
			colliders: self.colliders.clone(),
			impulse_joints: self.impulse_joints.clone(),
			multibody_joints: self.multibody_joints.clone(),
		}
	}
	pub fn build_body_states(&self, bodies_to_exclude: Vec<RigidBodyHandle>) -> BodyStates {// TODO: only send the states of vehicles, wheel mounts and wheels
		let mut out = BodyStates::new();
		for (handle, body) in self.bodies.iter() {
			if !bodies_to_exclude.contains(&handle) {
				out.insert(handle, BodyStateSerialize::from_rapier_body(body, None));
			}
		}
		out
	}
	pub fn build_body_creation_deletion_context(&mut self) -> RapierBodyCreationDeletionContext {
		RapierBodyCreationDeletionContext {
			bodies: &mut self.bodies,
			colliders: &mut self.colliders,
			islands: &mut self.islands
		}
	}
}

/// Only what is needed to init the client's rapier context, this should only be sent once when the client initially connects as it can be very large when serialized
#[derive(Serialize, Deserialize, Clone)]
#[cfg(feature = "debug_render_physics")]
pub struct PhysicsStateSend {
	pub islands: IslandManager,
	pub bodies: RigidBodySet,
	pub colliders: ColliderSet,
	pub impulse_joints: ImpulseJointSet,
	pub multibody_joints: MultibodyJointSet,
}

#[cfg(feature = "debug_render_physics")]
impl PhysicsStateSend {
	#[cfg(feature = "client")]
	pub fn init_bevy_rapier_context(&self, context: &mut RapierContext) {
		context.islands = self.islands.clone();
		context.bodies = self.bodies.clone();
		context.colliders = self.colliders.clone();
		context.impulse_joints = self.impulse_joints.clone();
		context.multibody_joints = self.multibody_joints.clone();
	}
	pub fn build_body_creation_deletion_context(&mut self) -> RapierBodyCreationDeletionContext {
		RapierBodyCreationDeletionContext {
			bodies: &mut self.bodies,
			colliders: &mut self.colliders,
			islands: &mut self.islands
		}
	}
}

/// Instead of sending over most of the rapier context, this just uses body handles and their corresponding states, only used for the `debug_render_physics` feature
#[cfg(any(feature = "server", feature = "debug_render_physics"))]
pub type BodyStates = HashMap<RigidBodyHandle, BodyStateSerialize>;

/// These are sent between the thread running `World::main_loop()` and thread running the Renet server
pub mod async_messages {
	use super::*;
	/// From the server to the simulation
	pub enum ToWorld {
		ClientUpdate(ClientUpdate),
		Pause,
		Play,
		TogglePlaying,
		CreateChunk(ChunkRef),
		RecoverVehicleFromFlip(ClientAuth)
	}
	/// From the simulation to the server
	pub enum FromWorld {
		State(WorldSend),
		Error(String)
	}
}

/// Loaded from and saved to the disk
#[derive(Serialize, Deserialize)]
pub struct WorldSave {
	/// Name of map file
	pub map: String,
	pub vehicles: HashMap<String, VehicleSave>,
	pub age: f64,
	pub playing: bool,
	pub gravity: Float
}

/// World state, sent to the client(s)
#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "client", derive(Resource, Default))]
pub struct WorldSend {
	/// Map of usernames and the state of their vehicles
	pub vehicles: HashMap<String, VehicleSend>,
	/// Current age of the world simulation in seconds
	pub age: f64,
	/// Whether the simulation is playing or paused
	pub playing: bool,
	/// Current server frames per second
	pub fps: f64,
	/// Rapier body states
	#[cfg(feature = "debug_render_physics")]
	pub body_states: BodyStates
}

#[cfg(feature = "client")]
impl WorldSend {
	#[cfg(feature = "debug_render_physics")]
	pub fn update_bevy_rapier_context(&self, context: &mut RapierContext, map_body_handle_opt: Option<RigidBodyHandle>) {
		for (handle, body_state) in self.body_states.iter() {
			if let Some(map_body_handle) = map_body_handle_opt {
				if handle == &map_body_handle {
					continue
				}
			}
			body_state.init_rapier_body(context.bodies.get_mut(*handle).expect("Unable to get body with handle from WorldSend::body_states"));
		}
	}
}

/// Main game simulation, not directly serializable
#[cfg(feature = "server")]
pub struct World {
	/// The map in use by this world
	pub map: ServerMap,
	/// Map of usernames and corresponding vehicles
	pub vehicles: HashMap<String, Vehicle>,
	/// Age of simulation
	pub age: Duration,
	/// Whether the simulation is playing or paused
	pub playing: bool,
	/// Gravity, should be negative
	pub gravity: Float,
	physics_state: PhysicsState
}

#[cfg(feature = "server")]
impl World {
	pub fn load(name: &str) -> Result<Self, String> {
		// name: name of world file
		let save = to_string_err_with_message(load_world(name), &format!("Could not load/deserialize world save file"))?;
		let map: ServerMap = ServerMap::from_save(to_string_err_with_message(load_map_metadata(&save.map), &format!("Could not load SaveMap (\"{}\") for world \"{}\"", &save.map, name))?)?;
		// Create blank physics state
		let mut physics_state: PhysicsState = PhysicsState::new(save.gravity);
		// Load vehicles
		let mut vehicles: HashMap<String, Vehicle> = HashMap::new();
		for (user, v_save) in save.vehicles.iter() {
			let v_static: VehicleStatic = to_string_err_with_message(load_static_vehicle(&v_save.type_), &format!("Could not load VehicleStatic \"{}\"", &v_save.type_))?;// TODO: Move this error msg to `load_static_vehicle`
			vehicles.insert(user.clone(), to_string_err_with_message(Vehicle::build(v_save, Rc::new(v_static), &mut physics_state.bodies, &mut physics_state.colliders, &mut physics_state.impulse_joints), &format!("Could not build Vehicle from VehicleStatic \"{}\"", &v_save.type_))?);
		}
		// Create world
		let mut out = Self {
			map,
			vehicles,
			age: Duration::from_secs_f64(save.age),
			playing: save.playing,
			gravity: save.gravity,
			physics_state
		};
		// Init map
		out.map.generic.init_rapier(&mut out.physics_state.bodies);
		// Load all necessary chunks
		out.load_unload_chunks();
		// debug
		/*{
			let serialized = bincode::serialize(&out.send(60.0)).unwrap();
			println!("Length of serialized WorldSend: {}", serialized.len());// 496
		}
		{
			let serialized = bincode::serialize(&out.build_static_data()).unwrap();
			println!("Length of serialized StaticData: {}", serialized.len());// 15798550 :( renet doesn't like
			//out.build_static_data().debug_print_sizes();
		}*/
		Ok(out)
	}
	pub fn from_map_name(map: &str) -> Result<Self, String> {
		Ok(Self {
			map: ServerMap::from_save(to_string_err(load_map_metadata(map))?)?,
			vehicles: HashMap::<String, Vehicle>::new(),
			age: Duration::new(0, 0),
			playing: false,
			gravity: -9.81,
			physics_state: PhysicsState::new(-9.81)
		})
	}
	pub fn new(
		map: ServerMap,
		vehicles: HashMap::<String, Vehicle>,
		playing: bool,
		gravity: Float
	) -> Self {
		Self {
			map,
			vehicles,
			age: Duration::new(0, 0),
			playing,
			gravity,
			physics_state: PhysicsState::new(gravity)
		}
	}
	pub fn main_loop(&mut self, rx: mpsc::Receiver<async_messages::ToWorld>, tx: mpsc::Sender<async_messages::FromWorld>) {
		// Init timing
		let mut prev_t: Instant = Instant::now();
		let mut dt: Duration;
		let mut dt_f64: f64;
		let mut fps: f64;
		let max_step: f64 = 0.1;// Maximum time step, to prevent collision glitches
		// Main loop, I want this to run as FAST as possible
		loop {
			// Timing, DO NOT USE dt, USE dt_f64 INSTEAD
			dt = prev_t.elapsed();
			prev_t = Instant::now();
			dt_f64 = dt.as_secs_f64();
			fps = 1.0 / dt_f64;// Do not use for simulation, only for performance measurement
			// max step
			if dt_f64 > max_step {
				dt_f64 = max_step;
			}
			// Simulation iteration
			self.step(dt_f64, fps, &rx, &tx);
			// Placeholder to prevent the computer from freezing
			thread::sleep(Duration::from_millis(5));
		}
	}
	/// Steps entire world simulation
	/// `dt_f64` and `fps` are not redundant because `fps` is NOT USED for the simulation, only to report performance. `dt_f64` is used used for the actual timestep.
	pub fn step(&mut self, dt_f64: f64, fps: f64, rx: &mpsc::Receiver<async_messages::ToWorld>, tx: &mpsc::Sender<async_messages::FromWorld>) {
		// 1: Updates
		// rx
		loop {
			match rx.try_recv() {// https://doc.rust-lang.org/stable/std/sync/mpsc/struct.Receiver.html
				Ok(update) => {
					match update {
						async_messages::ToWorld::ClientUpdate(update) => match self.vehicles.get_mut(&update.auth.name) {
							Some(v) => v.update_user_input(update.input),
							None => ()
						}
						async_messages::ToWorld::Pause => {self.playing = false},
						async_messages::ToWorld::Play => {self.playing = true},
						async_messages::ToWorld::TogglePlaying => {self.playing = !self.playing},
						async_messages::ToWorld::CreateChunk(ref_) => {
							//println!("Creating chunk {:?} upon client request", &ref_);
							self.map.load_or_create_chunk(&ref_);
						},
						async_messages::ToWorld::RecoverVehicleFromFlip(auth) => {
							match self.vehicles.get_mut(&auth.name) {
								Some(v) => v.recover_from_flip(&mut self.physics_state),
								None => {}
							}
						}
					}
				},
				Err(error) => {
					match error {
						mpsc::TryRecvError::Empty => break,
						mpsc::TryRecvError::Disconnected => panic!("Client updates message queue has been disconnected")
					}
				}
			}
		}
		// tx
		tx.send(async_messages::FromWorld::State(self.send(fps))).unwrap();
		// 2: Step physics simulation
		if self.playing {
			// All vehicle physics controllers
			for (_, v) in self.vehicles.iter_mut() {
				v.update_physics(dt_f64 as Float, 1.225, &mut self.physics_state, &self.map.path_set, self.gravity);
			}
			// Rapier
			self.physics_state.step();
		}
		// 3: Chunks
		self.load_unload_chunks();
		self.map.check_chunk_creator(&mut self.physics_state.build_body_creation_deletion_context());
	}
	pub fn load_unload_chunks(&mut self) {
		// Possibly similar to client::chunk_manager::RenderDistance::load_unload_chunks()
		// except this only loads chunks adjacent to the ones which vehicles are on
		let chunks_needed: Vec<ChunkRef> = self.chunks_needed();
		// 1: Unload
		self.map.generic.unload_chunks(&chunks_needed, &mut self.physics_state.build_body_creation_deletion_context());
		// 2: Load
		for chunk_ref in chunks_needed {
			if !self.map.generic.is_chunk_loaded(&chunk_ref) {
				//println!("Loading/Creating chunk {:?}", &chunk_ref);
				match self.map.load_or_create_chunk(&chunk_ref) {// Will be None if the chunk doesn't exist and the map auto chunk gen is disabled
					Some(chunk) => {self.map.generic.insert_chunk(chunk, Some(&mut self.physics_state.build_body_creation_deletion_context()));},
					None => {}
				}
			}
		}
	}
	fn chunks_needed(&self) -> Vec<ChunkRef> {
		// Get list of occupied chunks
		let mut needed_chunks = Vec::<ChunkRef>::new();
		for (_, v) in self.vehicles.iter() {
			let v_body_state = v.create_serialize_state(&self.physics_state.bodies, &self.map.path_set);
			needed_chunks.push(ChunkRef::from_world_point(v3_to_v2(&v_body_state.position.translation.vector), self.map.generic.chunk_size));
		}
		// Get adjacent chunks
		let mut additional_chunks = Vec::<ChunkRef>::new();
		for chunk_ref in &needed_chunks {
			additional_chunks.append(&mut chunk_ref.adjacent_chunks(self.map.generic.chunk_size, false));
		}
		needed_chunks.append(&mut additional_chunks);
		remove_dups(&mut needed_chunks);
		needed_chunks
	}
	pub fn save(&self) -> WorldSave {
		// save vehicles
		let mut save_vehicles = HashMap::new();
		for (user, vehicle) in self.vehicles.iter() {
			save_vehicles.insert(user.to_owned(), vehicle.save(&self.physics_state.bodies, &self.map.path_set));
		}
		WorldSave {
			map: self.map.generic.name.clone(),
			vehicles: save_vehicles,
			age: self.age.as_secs_f64(),
			playing: self.playing,
			gravity: self.gravity
		}
	}
	pub fn send(&self, fps: f64) -> WorldSend {
		// save vehicles
		let mut send_vehicles = HashMap::new();
		for (user, vehicle) in self.vehicles.iter() {
			send_vehicles.insert(user.to_owned(), vehicle.send(&self.physics_state.bodies, &self.map.path_set));
		}
		WorldSend {
			vehicles: send_vehicles,
			age: self.age.as_secs_f64(),
			playing: self.playing,
			fps,
			#[cfg(feature = "debug_render_physics")]
			body_states: self.physics_state.build_body_states(vec![])//self.map.body_handle_opt.expect("Could not get map body handle when sending World")])
		}
	}
	pub fn build_static_data(&self) -> StaticData {
		// build static vehicles
		let mut static_vehicles: HashMap<String, VehicleStatic> = HashMap::new();
		for (_, vehicle) in self.vehicles.iter() {
			static_vehicles.insert(vehicle.static_.type_name.clone(), (*vehicle.static_).clone());
		}
		// Physics send
		#[cfg(feature = "debug_render_physics")]
		let mut partial_physics = self.physics_state.send(/*&self.vehicles*/);
		// Done
		StaticData {
			map: self.map.send(#[cfg(feature = "debug_render_physics")] &mut partial_physics),// Need to remove map colliders from physics send as they are redundant and makes the StaticData very large
			static_vehicles,
			#[cfg(feature = "debug_render_physics")]
			partial_physics
		}
	}
}

#[cfg(all(test, feature = "server"))]
mod tests {
	use crate::vehicle::VehiclePathBoundController;
	use super::*;
	fn path_physics_initial_state() -> World {
		let generic_map = GenericMap::new("test-map", 100, 1, [0; 3]);
		let path_set = PathSet {
			generic: GenericPathSet {
				query_grid_scale: 0,
				routes: GenericDataset::new(),
				intersections: GenericDataset::new()
			},
			query_grid: HashMap::new(),
			paths: GenericDataset::<Path>{items: vec![
				(
					GenericRef::id(0),
					Path {
						generic: GenericPath {
							name: "test-path-name".to_owned(),
							knot_points: vec![
								V3::new(0.0, 0.0, 0.0),
								V3::new(0.0, 0.0, 50.0),
								V3::new(0.0, 10.0, 100.0),
								V3::new(0.0, 10.0, 150.0),
							],
							tangent_offsets: vec![
								V3::zeros(); 4
							],
							loop_: true
						},
						type_: Arc::new(PathType {
							ref_: "test-path-type-ref".to_owned(),
							name: "test-path-type-name".to_owned(),
							width: 10.0
						})
					}
				)
			]}
		};
		let map = ServerMap {
			generic: generic_map,
			path_set: path_set,
			gen: MapGenerator::RandomGen(map_generation::Random::flat()),
			chunk_creator: crate::map::ChunkCreationManager::new(1.0, 1)
		};
		let mut vehicles = HashMap::<String, Vehicle>::new();
		let v_static_rc = Rc::new(
			VehicleStatic {
				type_name: "test-vehicle-type".to_owned(),
				mass: 100.0,
				ctr_g_hight: 0.0,
				drag: 0.0,
				cross_sectional_area: 1.0,
				wheels: Vec::new()
		});
		vehicles.insert("test-user".to_owned(), Vehicle {
			static_: v_static_rc.clone(),
			latest_forces: None,
			path_forces_opt: None,
			latest_input: None,
			latest_input_t: 0,
			physics_controller: Box::new(VehiclePathBoundController::build(
				PathBoundBodyState {
					path_query: GenericQuery::<Path>::id(0),
					pos: PathPosition::new(0, 0.5),
					velocity: 10.0,
					forward: true,
					route_query_opt: None
				},
				v_static_rc
			))
		});
		// Done
		World::new(map, vehicles, true, -9.81)
	}
	#[test]
	fn path_physics() {
		// Initial state
		let mut world = path_physics_initial_state();
		let (_to_world_tx, to_world_rx) = mpsc::channel::<async_messages::ToWorld>();
		let (from_world_tx, _from_world_rx) = mpsc::channel::<async_messages::FromWorld>();
		// Test
		// TODO
		world.step(1.0, 0.0, &to_world_rx, &from_world_tx);
		let binding = world.send(0.0);
		#[allow(unused)]
		let v_state: &BodyStateSerialize = &binding.vehicles.get("test-user").unwrap().body_state;
		//assert_eq!(v_state.lin_vel, V3::new(0.0, 0.0, 10.0));
		//assert_eq!(v_state.position.translation.vector, V3::new(0.0, 0.0, 15.0));
	}
}