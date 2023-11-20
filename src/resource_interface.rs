// For loading & saving resources

use std::{error::Error, fs, collections::HashMap};
use std::io::Error as IoError;
use serde_json;
use extras;

use crate::{vehicle::VehicleStatic, ClientAuth, world::*, map::{Map, chunk::{Chunk, ChunkRef, RegularElevationMesh, Gen}}};
#[cfg(feature = "frontend")]
use crate::client::hardware_controller::Calibration;

// STATICS
static RESOURCES_DIR: &str = "../resources";
static VEHICLES_DIR: &str = "../resources/vehicles/";
static MAPS_DIR: &str = "../resources/maps/";
static WORLDS_DIR: &str = "../resources/worlds/";
static PATHS_DIR: &str = "../resources/paths/";
static MAP_GENERATORS_DIR: &str = "../resources/map_generators/";
static USERS_FILE: &str = "../resources/server_credentials.json";
static CLIENT_LOGIN_FILE: &str = "../resources/client_credentials.json";
static CALIBRATION_FILE: &str = "../resources/calibration.json";
static PORT_FILE: &str = "../resources/port.txt";

// Load
pub fn load_static_vehicle(name: &str) -> Result<VehicleStatic, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(&(VEHICLES_DIR.to_owned() + name + ".json"))?;
	let vehicle: VehicleStatic = serde_json::from_str(&raw_string)?;
	Ok(vehicle)
}

fn get_chunk_dir_path(chunk_ref: &ChunkRef, map_name: &str) -> String {
	return format!("{}{}/chunks/{}/", MAPS_DIR.to_owned(), map_name.to_owned(), chunk_ref.resource_dir_name())
}

pub fn list_created_chunks(map_name: &str) -> Result<Vec<ChunkRef>, String> {
	let mut out = Vec::<ChunkRef>::new();
	let chunks_dir = format!("{}/{}/chunks", MAPS_DIR.to_owned(), map_name);
	let entries = extras::to_string_err_with_message(fs::read_dir(&chunks_dir), &chunks_dir)?;
	for entry_res in entries {
		if let Ok(entry) = entry_res {
			let path = entry.path();
			// Check that it is a directory
			if !path.is_dir() {
				continue;//return Err(format!("There is a non-directory item in the `chunks` dir for the map \"{}\"", map_name));
			}
			// Attempt to interpret name
			out.push(ChunkRef::from_resource_dir_name(path.file_name().expect("Invalid path or no file/folder name found").to_str().expect("Name is not a valid UTF-8 string"))?);
		}
	}
	//println!("Created chunks: {:?}", &out);
	Ok(out)
}

pub fn load_chunk_texture(chunk_ref: &ChunkRef, map_name: &str) -> Result<Vec<u8>, Box<dyn Error>> {
	let data: Vec<u8> = fs::read(&(get_chunk_dir_path(chunk_ref, map_name) + "texture.png"))?;
	Ok(data)
}

pub fn load_chunk_data(chunk_ref: &ChunkRef, map_name: &str) -> Result<Chunk, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(&(get_chunk_dir_path(chunk_ref, map_name) + "data.json"))?;
	let chunk: Chunk = serde_json::from_str(&raw_string)?;
	Ok(chunk)
}

pub fn load_map_metadata(name: &str) -> Result<Map, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(&(MAPS_DIR.to_owned() + name + "/metadata.json"))?;
	let map: Map = serde_json::from_str(&raw_string)?;
	Ok(map)
}

// Is now part of map metadata
/*pub fn load_map_gen(name: &str) -> Result<Gen, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(&(MAP_GENERATORS_DIR.to_owned() + name + ".json"))?;
	let map_gen: Gen = serde_json::from_str(&raw_string)?;
	Ok(map_gen)
}

pub fn choose_map_gen() -> Result<Gen, Box<dyn Error>> {
	load_map_gen(&extras::prompt("Map gen"))// TODO: picklist
}*/

pub fn load_world(name: &str) -> Result<WorldSave, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(&(WORLDS_DIR.to_owned() + name + ".json"))?;
	let world: WorldSave = serde_json::from_str(&raw_string)?;
	Ok(world)
}

#[cfg(feature = "frontend")]
pub fn load_hardware_calibration() -> Result<Calibration, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(CALIBRATION_FILE)?;
	let cal: Calibration = serde_json::from_str(&raw_string)?;
	Ok(cal)
}

pub fn load_port() -> Result<u16, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(PORT_FILE)?;
	Ok(raw_string.parse::<u16>()?)
}

// Save
pub fn save_map(map: &Map) -> Result<(), Box<dyn Error>> {
	let raw_string = serde_json::to_string(map)?;
	fs::write(&(MAPS_DIR.to_owned() + &map.name + ".json"), &raw_string)?;
	Ok(())
}

pub fn save_chunk_data(chunk: &Chunk, map_name: &str) -> Result<(), Box<dyn Error>> {
	// Create directory
	fs::create_dir(&get_chunk_dir_path(&chunk.ref_, map_name))?;
	// Serialize
	let raw_string: String = serde_json::to_string(chunk)?;
	// Save to file
	fs::write(get_chunk_dir_path(&chunk.ref_, map_name) + "data.json", &raw_string)?;
	Ok(())
}

// Misc
pub fn load_file_with_better_error(path: &str) -> Result<String, Box<dyn Error>> {// With help from ChatGPT because I'm lasy
	match fs::read_to_string(path) {
        Ok(contents) => Ok(contents),
        Err(err) => {
            // Combine the error with the path information
            Err(Box::new(IoError::new(err.kind(), format!("Error reading file \"{}\": {}", path, err))))
        }
    }
}

// New user
pub mod users {
	use super::*;
	pub fn new(name: &str, psswd: &str) -> Result<(), Box<dyn Error>> {
		let mut deserialized = load_file()?;
		deserialized.insert(name.to_owned(), psswd.to_owned());
		let new_raw = serde_json::to_string(&deserialized)?;
		fs::write(&(USERS_FILE.to_owned()), &new_raw)?;
		Ok(())
	}
	pub fn authenticate(name: &str, psswd: &str) -> Result<bool, Box<dyn Error>> {
		let users: HashMap<String, String> = load_file()?;
		Ok(match users.get(name) {
			Some(correct_psswd) => psswd == correct_psswd,
			None => false
		})
	}
	fn load_file() -> Result<HashMap<String, String>, Box<dyn Error>> {
		let raw = fs::read_to_string(&(USERS_FILE.to_owned()))?;
		let deserialized: HashMap<String, String> = serde_json::from_str(&raw)?;
		Ok(deserialized)
	}
	pub fn client_login() -> Result<ClientAuth, Box<dyn Error>> {
		let raw = fs::read_to_string(CLIENT_LOGIN_FILE)?;
		let deserialized: ClientAuth = serde_json::from_str(&raw)?;
		Ok(deserialized)
	}
}