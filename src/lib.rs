/*
Stationary bike-controlled video game / training program
Created by Hadrian Ward, 2023-6-8

2023-7-29: Major design change: Not using the web at all. Instead using bevy, and renet to send data back and forth
	The `frontend` feature is for anything only used by the client GUI and `backend` is only for the server/world simulation

2023-9-21: Just to clear things up, when the client connects to the game server, it is sent a bincode::serialize()ed instance of world::StaticData.
	After that only WorldSend is sent to the client as often as possible. Major change: A partial copy of the server's physics state will be sent upon initial
	connection (world::PhysicsStateSend) which means that RigidBodyHandles can be sent with body state information attached (world::BodyStates), and
	they will work with the client's rapier context

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
*/
#![allow(warnings)]// TODO: remove when I have a lot of free-time
use std::{env, ops};
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
use nalgebra::{Point3, Point2, Vector3, Vector2, point, Matrix, Const, ArrayStorage, OPoint, Translation};
#[cfg(feature = "frontend")]
use bevy::ecs::system::Resource;

// Modules
mod world;
mod map;
mod vehicle;
pub mod resource_interface;
#[cfg(feature = "frontend")]
pub mod client;
#[cfg(feature = "web_server")]
pub mod web_server;
#[cfg(feature = "backend")]
pub mod renet_server;

// Tests
#[cfg(test)]
mod tests;

// Extras
use extras;

use nalgebra::Isometry3;

// Prelude, added 2023-9-9
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
	// Misc, important to not use anything that is behind a feature
	pub use crate::{
		world::{StaticData, PhysicsState, World, WorldSave, WorldSend},
		map::{Map, path::{Path, PathSet, PathBoundBodyState, BCurve}, chunk::{Chunk, ChunkRef, RegularElevationMesh, Gen}},
		vehicle::{Vehicle, VehicleStatic, VehicleSave, VehicleSend, Wheel, WheelStatic, BodyStateSerialize, BodyPhysicsController, VehicleLinearForces},
		InputData,
		ClientAuth,
		ClientUpdate,
		EightWayDir,
		IntP2,
		resource_interface
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
		(diff[0].powi(2) + diff[1].powi(1) + diff[2].powi(2)).powf(0.5)
	}
	// Convert between 2D and 3D, important to note that the Y-value in 2D corresponds to the Z-value in 3D
	pub fn v3_to_v2(v: V3) -> V2 {
		V2::new(v[0], v[2])
	}
	pub fn p3_to_p2(p: P3) -> P2 {
		point![p[0], p[2]]
	}
	pub fn p2_to_p3(p: P2) -> P3 {
		point![p[0], 0.0, p[1]]
	}
}

use prelude::*;

// Structs/Enums
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct InputData {
	steering: f64,// -1 right to 1 left
	speed: f64,// Actual speed measured on bike
	power: f64,// Watts
	brake: f64// G-acceleration
}

impl InputData {
	pub fn to_string(&self) -> String {
		format!("Power: {:.0}W, Speed: {:.1} m/s, Steering: {:.2}, Brake G-acc: {:.2}", self.power, self.speed, self.steering, self.brake)
	}
}

#[derive(Serialize, Deserialize, Clone)]
#[cfg_attr(feature = "frontend", derive(Resource))]
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
	pub fn unit_displacement(&self) -> IntP2 {
        match self {
            Self::E  => IntP2( 1,  0),
            Self::NE => IntP2( 1,  1),
            Self::N  => IntP2( 0,  1),
            Self::NW => IntP2(-1,  1),
            Self::W  => IntP2(-1,  0),
            Self::SW => IntP2(-1, -1),
            Self::S  => IntP2( 0, -1),
            Self::SE => IntP2( 1, -1),
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
pub struct IntP2(pub Int, pub Int);

impl IntP2 {
	pub fn mult(&self, other: Int) -> Self {
		Self(self.0 * other, self.1 * other)
	}
}

impl ops::Add<IntP2> for IntP2 {
	type Output = IntP2;

	fn add(self, other: Self) -> Self {
		Self(self.0 + other.0, self.1 + other.1)
	}
}

impl ops::Sub<IntP2> for IntP2 {
	type Output = IntP2;

	fn sub(self, other: IntP2) -> Self {
		Self(self.0 - other.0, self.1 - other.1)
	}
}

//#[derive(Serialize, Deserialize)]
//pub struct UserDataBodyHandles(HashMap<u128, RigidBodyHandle>);

#[cfg(feature = "backend")]
pub fn ui_main() {
	// Parse arguments
	let args: Vec<String> = env::args().collect();
	if args.len() < 2 {// Just the program name, default to running the server GUI
		panic!("Not enough arguments, see source code");
	}
	else {
		match &args[1][..] {
			"-web-server" => {
				#[cfg(feature = "web_server")]
				{
					println!("WARNING: THE WEB INTERFACE IS KEPT ONLY FOR TESTING PURPOSES AND MAY NOT WORK ANYMORE");
					assert!(args.len() >= 3, "Not enough arguments");
					// Start WorldServer
					let mut server = web_server::WorldServer::init(&args[2]);
					println!("Running world");
					server.main_loop();
				}
				#[cfg(not(feature = "web_server"))]
				{
					println!("The web server will only work if the \"web_server\" feature is enabled");
				}
			},
			"-server" => {
				assert!(args.len() >= 3, "Not enough arguments");
				// Start server with renet
				let mut server = renet_server::WorldServer::init(&args[2]);
				println!("Running world");
				server.main_loop();
			},
			"-new-map" => {
				let name = extras::prompt("Name");
				let chunk_size = extras::prompt("Chunk size").parse::<UInt>().unwrap();
				let chunk_grid_size = extras::prompt("Number of points along each side of chunk (chunk size)").parse::<UInt>().unwrap();
				let gen = Gen::default();
				// name: &str, chunk_size: UInt, chunk_grid_size: UInt, gen: gen::Gen, background_color: [u8; 3]
				resource_interface::save_map(&Map::new(&name, chunk_size, chunk_grid_size, gen, [0, 128, 0])).unwrap();
			},
			"-new-user" => {
				if args.len() < 4 {
					panic!("Not enough arguments");
				}
				resource_interface::users::new(&args[2], &args[3]).unwrap();
				println!("Created user \"{}\" with password \"{}\"", &args[2], &args[3]);
			},
			_ => panic!("Invalid arguments")
		}
	}
}