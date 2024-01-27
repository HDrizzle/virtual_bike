/*
Stationary bike-controlled video game / training program
Created by Hadrian Ward, 2023-6-8

2023-7-29: Major design change: Not using the web at all. Instead using bevy, and renet to send data back and forth
	The `frontend` feature is for anything only used by the client GUI and `backend` is only for the server/world simulation

2023-9-21: Just to clear things up, when the client connects to the game server, it is sent a bincode::serialize()ed instance of world::StaticData.
	After that only WorldSend is sent to the client as often as possible. Major change: A partial copy of the server's physics state will be sent upon initial
	connection (world::PhysicsStateSend) which means that RigidBodyHandles can be sent with body state information attached (world::BodyStates), and
	they will work with the client's rapier context (2023-12-9: All of this will be behind the `debug_render_physics` feature)

2023-9-23: The previous change will now be behind the `#[cfg(feature = "debug_render_physics")]` feature

2023-9-29: I'm bored and I have decided to implement a new feature: Chunks, like in Minecraft. This allows the game world to be gigantic,
	but not use too much memory in either the client or server.

2023-10-7: There is a problem. The chunk system is incompatible with sending all of the body handles and positions over, as the server's loaded chunks
	may be different from the client's. Solution: have the "debug_render_physics" feature only send the states of vehicles, wheel mounts and wheels.

2023-11-3: changed tachometer time units to microseconds because I am now using an optical sensor with the trainer flywheel which
	is really fast and millisecs are just not precise enough

2023-11-6: I have temporarily given up on world gen, so I have created the new module: crate::map::path

2023-11-21: Major design change for the client: due to the difficulty of having different "states" in the bevy app requiring only certain systems to be used,
	Bevy will now only be used while actually playing the game. A plain egui app using eframe will be used for the login screen.

2023-11-25: Started work on generic chunks, meaning that a map can be configured so that newly explored chunks will be be copied from a template and not saved,
	this will save disk space for, for example, flat or repeating terrain.

2023-12-2: I will attempt to upgrade to the latest version of Bevy (bevy-^0.12) as well as everything that uses it.

2023-12-9: I decided to not use rapier in the client except when `debug_render_physics` is enabled
*/
#![allow(warnings)]// TODO: remove when I have a lot of free-time
use std::{fmt, env, ops, error::Error, collections::hash_map::DefaultHasher, hash::{Hash, Hasher}, time::{SystemTime, UNIX_EPOCH}, f32::consts::PI};
#[cfg(any(feature = "server", feature = "debug_render_physics"))]
use rapier3d::{dynamics::{RigidBodySet, IslandManager}, geometry::ColliderSet};
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
use nalgebra::{Point3, Point2, Vector3, Vector2, point, Matrix, Const, ArrayStorage, OPoint, Translation, Isometry3, UnitQuaternion};
#[cfg(feature = "client")]
use bevy::{prelude::Transform, ecs::system::Resource, render::{mesh::{Mesh, Indices}, render_resource::PrimitiveTopology}};
#[cfg(feature = "debug_render_physics")]
use bevy_rapier3d::plugin::RapierContext;
use dialoguer;
use rand::Rng;

// Modules
mod world;
mod map;
mod vehicle;
#[cfg(feature = "server")]
mod physics;
#[cfg(feature = "server")]
pub mod validity;
pub mod resource_interface;
#[cfg(feature = "client")]
pub mod client;
pub mod renet_server;

// Tests
#[cfg(test)]
mod tests;

// Prelude, added 2023-9-9
#[allow(unused)]
mod prelude {
	use super::*;
	// Name of this app
	pub const APP_NAME: &str = "Virtual Bike";
	// Types
	pub type Float = f32;
	pub type Int = i64;
	pub type UInt = u64;
	// Example of basic usage: https://rapier.rs/docs/user_guides/rust/introduction_to_nalgebra
	pub type P2 = Point2<Float>;
	pub type V2 = Vector2<Float>;
	pub type P3 = Point3<Float>;
	pub type V3 = Vector3<Float>;
	pub type Iso = Isometry3<Float>;
	pub const EPSILON: Float = 1.0e-6;// Arbitrary
	// Misc
	pub use crate::{
		world::{StaticData, WorldSave, WorldSend},
		map::{GenericMap, path::{Path, PathSet, PathBoundBodyState, PathPosition, BCurve, BCurveSample, BCURVE_LENGTH_ESTIMATION_SEGMENTS}, chunk::{Chunk, ChunkRef, RegularElevationMesh}},
		vehicle::{VehicleStatic, VehicleStaticModel, VehicleSave, VehicleSend, Wheel, WheelStatic, BodyStateSerialize, BodyForces},
		renet_server::{Request, Response},
		GenericError,
		InputData,
		ClientAuth,
		ClientUpdate,
		EightWayDir,
		IntV2,
		resource_interface,
		BasicTriMesh,
		SimpleIso,
		SimpleRotation,
		struct_subset
	};
	#[cfg(feature = "server")] pub use crate::{
		physics::{PhysicsController, PhysicsUpdateArgs, BodyAveragableState, defaut_extra_forces_calculator},
		vehicle::Vehicle,
		world::{World, PhysicsState},
		map::{ServerMap, SaveMap, map_generation::{MapGenerator, MeshCreationArgs, gis::WorldLocation}, chunk::{ChunkCreationArgs, ChunkCreationResult}},
		validity
	};
	#[cfg(any(feature = "server", feature = "debug_render_physics"))] pub use crate::{
		RapierBodyCreationDeletionContext,
		client::cache::CacheableBevyAsset
	};
	// Utility functions because nalgebra is friggin complicated
	pub fn add_isometries(iso1: &Iso, iso2: &Iso) -> Iso {
		// Adds two isometries together
		// example use: if iso1 is a vehicle position and iso2 is the position of a wheel wrt the body, this would return the wheel's global position
		Iso {
			rotation: iso1.rotation * iso2.rotation,
			translation: Translation::<Float, 3>::from(iso1.translation.vector + opoint_to_matrix(iso1.rotation.transform_point(&matrix_to_opoint(iso2.translation.vector))).vector)
		}
	}
	pub fn matrix_to_opoint(mat: Matrix<Float, Const<3>, Const<1>, ArrayStorage<Float, 3, 1>>) -> OPoint<Float, Const<3>> {
		point![mat[0], mat[1], mat[2]]
	}
	pub fn opoint_to_matrix(point: OPoint<Float, Const<3>>) -> Translation<Float, 3> {
		Translation::<Float, 3>::new(point.coords[0], point.coords[1], point.coords[2])
	}
	pub fn v3_dist(v0: V3, v1: V3) -> Float {
		let diff = v1 - v0;
		(diff[0].powi(2) + diff[1].powi(2) + diff[2].powi(2)).powf(0.5)
	}
	// Convert between 2D and 3D, important to note that the Y-value in 2D corresponds to the Z-value in 3D
	pub fn v3_to_v2(v: V3) -> V2 {
		V2::new(v[0], v[2])
	}
	pub fn v2_to_v3(v: V2) -> V3 {
		V3::new(v[0], 0.0, v[1])
	}
	pub fn p3_to_p2(p: P3) -> P2 {
		point![p[0], p[2]]
	}
	pub fn p2_to_p3(p: P2) -> P3 {
		point![p[0], 0.0, p[1]]
	}
	// Convert between nalgebra and bevy
	#[cfg(feature = "client")]
	pub fn nalgebra_quat_to_bevy_quat(quat: &UnitQuaternion<Float>) -> bevy::math::Quat {
		bevy::math::Quat::from_xyzw(quat.i, quat.j, quat.k, quat.w)
	}
	#[cfg(feature = "client")]
	pub fn nalgebra_vec3_to_bevy_vec3(v: &V3) -> bevy::math::Vec3 {
		bevy::math::Vec3 {
			x: v.x,
			y: v.y,
			z: v.z
		}
	}
	#[cfg(feature = "client")]
	pub fn nalgebra_iso_to_bevy_transform(iso: Iso) -> Transform {
		Transform {
			translation: nalgebra_vec3_to_bevy_vec3(&iso.translation.vector),
			rotation: nalgebra_quat_to_bevy_quat(&iso.rotation),
			scale: bevy::math::Vec3::ONE
		}
	}
	// Round float towards -inf
	pub fn round_float_towards_neg_inf(n: Float) -> Int {
		n.floor() as Int
	}
	pub fn mod_or_clamp(n: Int, max_u: UInt, loop_: bool) -> (Int, bool) {// With help from ChatGPT
		// If loop_: modulus on `n` / `max`, else: clamps to range [0, max]
		// Returns: whether looped or clamped
		let max = max_u as Int;
		if loop_ {
			let result = n.rem_euclid(max);
			(result, result != n)
		} else {
			let clamped: Int = n.max(0).min(max as Int - 1);
			(clamped, n != clamped as Int)
		}
	}
	// Copied from extras
	pub fn to_string_err<T, E: Error>(result: Result<T, E>) -> Result<T, String> {
		match result {
			Ok(t) => Ok(t),
			Err(e) => Err(e.to_string())
		}
	}
	pub fn to_string_err_with_message<T, E: Error>(result: Result<T, E>, message: &str) -> Result<T, String> {
		match result {
			Ok(t) => Ok(t),
			Err(e) => Err(format!("Message: {}, Error: {}", message, e.to_string()))
		}
	}
	pub fn prompt(s: &str) -> String {
		dialoguer::Input::new()
			.with_prompt(s)
			.interact_text()
			.unwrap()
	}
	pub fn rand_unit() -> f64 {
		//(rand::thread_rng().gen_range(0..1000000000) as f64) / 1000000000.0
		rand::thread_rng().gen()
	}
	pub fn remove_dups<T>(v: &mut Vec<T>)// From ChatGPT
	where
		T: PartialEq + Clone, // T needs to implement Clone for this approach
	{
		let mut unique_items = Vec::new();
		let mut index = 0;

		while index < v.len() {
			let item = v[index].clone(); // Clone the item for comparison

			if !unique_items.contains(&item) {
				unique_items.push(item.clone());
				index += 1;
			} else {
				v.remove(index);
			}
		}
	}
	pub fn calculate_hash<T: Hash>(t: &T) -> u64 {// https://doc.rust-lang.org/std/hash/index.html
		let mut s = DefaultHasher::new();
		t.hash(&mut s);
		s.finish()
	}
	pub fn get_unix_ts_secs_u64() -> u64 {
		let start = SystemTime::now();
		start.duration_since(UNIX_EPOCH).expect("Time went backwards").as_secs()
	}
}

use prelude::*;

// Generic error, from ChatGPT
#[derive(Debug)]
pub struct GenericError<T> {
    message: T,
}

impl<T: fmt::Display> fmt::Display for GenericError<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl<T: fmt::Debug + fmt::Display> Error for GenericError<T> {}

impl<T: fmt::Display> GenericError<T> {
    fn new(message: T) -> Self {
        GenericError { message }
    }
}

// Structs/Enums
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default)]
pub struct InputData {
	steering: Float,// -1 right to 1 left
	speed: Float,// Actual speed measured on bike
	power: Float,// Watts
	brake: Float// Acc (m/s^2)
}

impl InputData {
	pub fn to_string(&self) -> String {
		format!("Power: {:.0}W, Speed: {:.1} m/s, Steering: {:.2}, Brake G-acc: {:.2}", self.power, self.speed, self.steering, self.brake)
	}
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[cfg_attr(feature = "client", derive(Resource))]
pub struct ClientAuth {
	name: String,
	psswd: String
}

#[derive(Serialize, Deserialize)]
pub struct ClientUpdate {// JSON request POSTed to /play/client_update should be deserializable as this
	auth: ClientAuth,
	input: InputData
}

#[derive(Clone)]
pub enum EightWayDir {
	E,
	NE,
	N,
	NW,
	W,
	SW,
	S,
	SE
}

impl EightWayDir {
	pub fn from_num(n: usize) -> Result<Self, String> {
		match n {
			0 => Ok(Self::E ),
            1 => Ok(Self::NE),
            2 => Ok(Self::N ),
            3 => Ok(Self::NW),
            4 => Ok(Self::W ),
            5 => Ok(Self::SW),
            6 => Ok(Self::S ),
            7 => Ok(Self::SE),
			invalid => Err(format!("Number for EightWayDir has to be within [0, 7], instead it is {}", invalid))
		}
	}
	pub fn to_num(&self) -> usize {
		match self {
            Self::E  => 0,
            Self::NE => 1,
            Self::N  => 2,
            Self::NW => 3,
            Self::W  => 4,
            Self::SW => 5,
            Self::S  => 6,
            Self::SE => 7,
        }
	}
	pub fn unit_displacement(&self) -> IntV2 {
        match self {
            Self::E  => IntV2( 1,  0),
            Self::NE => IntV2( 1,  1),
            Self::N  => IntV2( 0,  1),
            Self::NW => IntV2(-1,  1),
            Self::W  => IntV2(-1,  0),
            Self::SW => IntV2(-1, -1),
            Self::S  => IntV2( 0, -1),
            Self::SE => IntV2( 1, -1),
        }
    }
	pub fn is_straight(&self) -> bool {
		self.to_num() % 2 == 0
	}
	pub fn iter() -> EightWayDirIter {
		EightWayDirIter{next: 0}
	}
}

pub struct EightWayDirIter {
	next: usize
}

impl Iterator for EightWayDirIter {
	type Item = EightWayDir;
	fn next(&mut self) -> Option<Self::Item> {
		if self.next >= 8 {
			return None
		}
		let out = EightWayDir::from_num(self.next).unwrap();
		self.next += 1;
		Some(out)
	}
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Debug, Eq, Hash)]
pub struct IntV2(pub Int, pub Int);

impl IntV2 {
	pub fn mult(&self, other: Int) -> Self {
		Self(self.0 * other, self.1 * other)
	}
	pub fn to_v2(&self) -> V2 {
		V2::new(self.0 as Float, self.1 as Float)
	}
}

impl ops::Add<IntV2> for IntV2 {
	type Output = IntV2;

	fn add(self, other: Self) -> Self {
		Self(self.0 + other.0, self.1 + other.1)
	}
}

impl ops::Sub<IntV2> for IntV2 {
	type Output = IntV2;

	fn sub(self, other: IntV2) -> Self {
		Self(self.0 - other.0, self.1 - other.1)
	}
}

pub struct BasicTriMesh {
	pub vertices: Vec<P3>,
	pub indices: Vec<[u32; 3]>
}

impl BasicTriMesh {
	#[cfg(feature = "client")]
	pub fn build_bevy_mesh(&self) -> Mesh {
		// Check if valid
		self.is_valid().unwrap();
		// Start with "blank" mesh
		let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
		let mut vertex_arrays = Vec::<[f32; 3]>::new();
		for v in &self.vertices {
			vertex_arrays.push([v[0], v[1], v[2]]);
		}
		mesh.insert_attribute(
			Mesh::ATTRIBUTE_POSITION,
			vertex_arrays,
		);
		mesh.set_indices(Some(Indices::U32(self.flatten_and_reverse_indices())));
		// Done
		mesh
	}
	pub fn is_valid(&self) -> Result<(), String> {
		// Check that all indices are within limits
		for triangle in &self.indices {
			for index in triangle {
				if index >= &(self.vertices.len() as u32) {
					return Err(format!("BasicTriMesh::is_valid(): index out of bounds"));
				}
			}
		}
		// Done
		Ok(())
	}
	pub fn flatten_and_reverse_indices(&self) -> Vec<u32> {
		let mut out = Vec::<u32>::new();
		for set in &self.indices {
			out.push(set[0]);
			out.push(set[2]);// Not a mistake
			out.push(set[1]);// Triangle winding or something, do not change
		}
		out
	}
}

impl Default for BasicTriMesh {
	fn default() -> Self {
		Self {
			vertices: Vec::new(),
			indices: Vec::new()
		}
	}
}

#[cfg(any(feature = "server", feature = "debug_render_physics"))]
pub struct RapierBodyCreationDeletionContext<'a> {
	pub bodies: &'a mut RigidBodySet,
	pub colliders: &'a mut ColliderSet,
	pub islands: &'a mut IslandManager
}

#[cfg(any(feature = "server", feature = "debug_render_physics"))]
impl<'a> RapierBodyCreationDeletionContext<'a> {// From ChatGPT
    #[cfg(feature = "debug_render_physics")]
    pub fn from_bevy_rapier_context(ctx: &'a mut RapierContext) -> Self {
        Self {
            bodies: &mut ctx.bodies,
            colliders: &mut ctx.colliders,
            islands: &mut ctx.islands,
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub struct SimpleIso {// Isometry with just
	translation: V3,
	rotation: SimpleRotation
}

impl SimpleIso {
	pub fn to_iso(&self) -> Iso {
		Iso {
			translation: Translation{vector: self.translation},
			rotation: self.rotation.to_quat()
		}
	}
	pub fn from_iso(iso: Iso) -> Self {
		Self {
			translation: iso.translation.vector,
			rotation: SimpleRotation::from_quat(&iso.rotation)
		}
	}
}

#[derive(Clone, PartialEq, Debug)]
pub struct SimpleRotation {
	pub yaw: Float,
	pub pitch: Float
}

impl SimpleRotation {
	pub fn to_quat(&self) -> UnitQuaternion<Float> {
		UnitQuaternion::from_axis_angle(&V3::y_axis(), self.yaw) * UnitQuaternion::from_axis_angle(&V3::x_axis(), self.pitch)
	}
	pub fn from_quat(quat: &UnitQuaternion<Float>) -> Self {
		// Pitch
		let y = quat.transform_vector(&V3::new(0.0, 0.0, 1.0)).y;
		let pitch = if y - 1.0 < EPSILON {// TODO: fix
			PI/2.0
		}
		else {
			if y + 1.0 < EPSILON {
				-PI/2.0
			}
			else {
				y.asin()
			}
		} - PI/2.0;
		let (_roll, _pitch, yaw) = quat.euler_angles();// https://docs.rs/nalgebra/latest/nalgebra/geometry/type.UnitQuaternion.html#method.euler_angles
		Self {
			yaw,
			pitch
		}
	}
}

#[macro_export]
macro_rules! struct_subset {
	($struct_name:ident, $new_struct_name:ident, $($field:ident),+) => {
		#[derive(Serialize, Deserialize, Clone)]
		pub struct $new_struct_name {
			pub $($field: String),+
		}
	};
}

#[cfg(feature = "server")]
pub fn ui_main() {
	// Parse arguments
	let args: Vec<String> = env::args().collect();
	if args.len() < 2 {// Just the program name, default to running the server GUI
		panic!("Not enough arguments, see crate::ui_maine()");
	}
	else {
		match &args[1][..] {
			"-server" => {
				assert!(args.len() >= 3, "Not enough arguments");
				// Start server with renet
				let mut server = renet_server::WorldServer::init(&args[2], args.contains(&"-localhost".to_string()));
				println!("Running world");
				server.main_loop();
			},
			"-new-map" => {
				let name = prompt("Name");
				let chunk_size = prompt("Chunk size").parse::<UInt>().unwrap();
				let chunk_grid_size = prompt("Number of points along each side of chunk (chunk size)").parse::<UInt>().unwrap();
				let gen = MapGenerator::default();
				// name: &str, chunk_size: UInt, chunk_grid_size: UInt, gen: gen::Gen, background_color: [u8; 3]
				resource_interface::save_map(&ServerMap::new(&name, chunk_size, chunk_grid_size, gen, [0, 128, 0]).save()).unwrap();
			},
			"-new-user" => {
				if args.len() < 4 {
					panic!("Not enough arguments");
				}
				resource_interface::users::new(&args[2], &args[3]).unwrap();
				println!("Created user \"{}\" with password \"{}\"", &args[2], &args[3]);
			},
			"-validity-test" => {
				validity::main_ui();
			}
			_ => panic!("Invalid arguments")
		}
	}
}