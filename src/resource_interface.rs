//! For loading & saving resources

use std::{error::Error, fs, collections::HashMap};
use std::io::Error as IoError;
use serde_json;

use crate::map::path::PathType;
use crate::{
	prelude::*,
	map::chunk::ChunkDirComponent
};
#[cfg(feature = "client")]
use crate::client::hardware_controller::Calibration;

// STATICS
pub static RESOURCES_DIR: &str = "../resources";
pub static VEHICLES_DIR: &str = "../resources/vehicles/";
pub static MAPS_DIR: &str = "../resources/maps/";
pub static WORLDS_DIR: &str = "../resources/worlds/";
pub static PATH_TYPES_DIR: &str = "../resources/path_types/";
pub static MAP_GENERATORS_DIR: &str = "../resources/map_generators/";
pub static PATH_TEXTURES_DIR: &str = "../resources/path_textures/";
pub static USERS_FILE: &str = "../resources/server_credentials.json";
pub static CLIENT_LOGIN_FILE: &str = "../resources/client_credentials.json";
pub static CALIBRATION_FILE: &str = "../resources/calibration.json";
pub static PORT_FILE: &str = "../resources/port.txt";
pub static CLIENT_SETTINGS_FILE: &str = "../resources/client_settings.json";

pub static VEHICLE_STATIC_JSON_FILENAME: &str = "static_data.json";

/*
fn get_chunk_dir_path(chunk_ref: &ChunkRef, map_name: &str) -> String {
	format!("{}{}/chunks/{}/", MAPS_DIR.to_owned(), map_name.to_owned(), chunk_ref.resource_dir_name())
}*/

pub fn list_created_chunks(map_name: &str) -> Result<Vec<ChunkRef>, String> {
	let mut out = Vec::<ChunkRef>::new();
	let chunks_dir = format!("{}/{}/chunks", MAPS_DIR.to_owned(), map_name);
	let entries = to_string_err_with_message(fs::read_dir(&chunks_dir), &chunks_dir)?;
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

// Load
pub fn load_static_vehicle_gltf(name: &str) -> Result<Vec<u8>, String> {
	let path = &(VEHICLES_DIR.to_owned() + name + "/model.glb");
	to_string_err_with_message(fs::read(path), &format!("Failed to load 3D GLB file at {} for type {}", path, name))
}

pub fn load_static_vehicle(name: &str) -> Result<VehicleStatic, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(&(VEHICLES_DIR.to_owned() + name + "/" + VEHICLE_STATIC_JSON_FILENAME))?;
	let vehicle: VehicleStatic = serde_json::from_str(&raw_string)?;
	Ok(vehicle)
}

#[cfg(feature = "server")]
/// Finds path of given chunk ref, if successful returns: (Path, whether it is a generic chunk)
pub fn find_chunk(chunk_ref: &ChunkRef, map_name: &str, required_components: &Vec<ChunkDirComponent>) -> Result<(String, bool), String> {
	// Tries everything to find path to chunk dir
	let path_options: Vec<(bool, Option<String>)> = vec![
		(false, does_non_generic_chunk_exist(chunk_ref, map_name)),// 1. Test if it exists as a normal chunk
		(true, does_generic_chunk_exist(map_name))// 2. Is there a generic chunk to use instead?
	];
	// Loop over options
	for (generic, opt) in path_options {
		if let Some(dir_path) = opt {
			//dbg!(&dir_path);
			let mut component_missing = false;
			for requirement in required_components {
				if !requirement.exists(&dir_path) {
					component_missing = true;
				}
			}
			if !component_missing {
				return Ok((dir_path, generic));
			}
		}
	}
	/*if let Some(path) = does_non_generic_chunk_exist(chunk_ref, map_name) {
        return Ok((path, false));
    }
	if let Some(path) = does_generic_chunk_exist(map_name) {
        return Ok((path, true));
    }*/
	// The chunk does not exist
	Err(format!("Could not find regular or generic chunk for map \"{}\" with ref: {:?} with requirements: {:?}", map_name, chunk_ref, required_components))
}

#[cfg(feature = "server")]
pub fn does_generic_chunk_exist(map_name: &str) -> Option<String> {
	// Tests if map has generic chunk, if so returns path to it
	let path = ChunkRef{position: IntV2(0, 0)}.resource_dir(map_name, true);
	if fs::read_dir(path.clone()).is_ok() {
		Some(path)
	}
	else {
		None
	}
}

#[cfg(feature = "server")]
pub fn does_non_generic_chunk_exist(chunk_ref: &ChunkRef, map_name: &str) -> Option<String> {
	let path = chunk_ref.resource_dir(map_name, false);
	if fs::read_dir(path.clone()).is_ok() {
		Some(path)
	}
	else {
		None
	}
}

#[cfg(feature = "server")]
pub fn load_chunk_texture(chunk_ref: &ChunkRef, map_name: &str) -> Result<(Vec<u8>, bool), String> {
	let texture_component = ChunkDirComponent::Texture;
	let (dir_path, generic): (String, bool) = find_chunk(chunk_ref, map_name, &vec![texture_component.clone()])?;
	//dbg!(&dir_path);
	//dbg!(texture_component.exists(&dir_path));
	let path: String = dir_path + &texture_component.file_name();
	let data: Vec<u8> = to_string_err_with_message(fs::read(&path), &format!("Attempt to load \"{}\"", &path))?;
	Ok((data, generic))
}

#[cfg(feature = "server")]
pub fn load_chunk_data(chunk_ref: &ChunkRef, map_name: &str) -> Result<Chunk, Box<dyn Error>> {
	let path: String = find_chunk(chunk_ref, map_name, &vec![ChunkDirComponent::JsonData])?.0;
	let raw_string: String = load_file_with_better_error(&(path + "data.json"))?;
	let mut chunk: Chunk = serde_json::from_str(&raw_string)?;
	// Set chunk's position because the generic one will likely be incorrect
	chunk.set_position(chunk_ref);
	// Done
	Ok(chunk)
}

#[cfg(feature = "server")]
pub fn load_map_metadata(name: &str) -> Result<SaveMap, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(&(MAPS_DIR.to_owned() + name + "/metadata.json"))?;
	let save_map: SaveMap = serde_json::from_str(&raw_string)?;
	Ok(save_map)
}

#[cfg(feature = "client")]
pub fn load_client_settings() -> Result<crate::client::Settings, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(CLIENT_SETTINGS_FILE)?;
	let settings: crate::client::Settings = serde_json::from_str(&raw_string)?;
	Ok(settings)
}

pub fn load_world(name: &str) -> Result<WorldSave, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(&(WORLDS_DIR.to_owned() + name + ".json"))?;
	let world: WorldSave = serde_json::from_str(&raw_string)?;
	Ok(world)
}

pub fn load_path_type(name: &PathTypeRef) -> Result<PathType, Box<dyn Error>> {
	let raw_string: String = load_file_with_better_error(&(PATH_TYPES_DIR.to_owned() + name + ".json"))?;
	let path_type: PathType = serde_json::from_str(&raw_string)?;
	Ok(path_type)
}

pub fn load_path_texture(name: &PathTypeRef) -> Result<Vec<u8>, String> {
	let path = format!("{}{}.png", PATH_TEXTURES_DIR, name);
	to_string_err_with_message(fs::read(&path), &format!("Attempt to load \"{}\"", &path))
}

#[cfg(feature = "client")]
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
#[cfg(feature = "server")]
pub fn save_map(map: &SaveMap) -> Result<(), Box<dyn Error>> {
	let raw_string = serde_json::to_string(map)?;
	fs::write(&(MAPS_DIR.to_owned() + &map.generic.name + ".json"), &raw_string)?;
	Ok(())
}

#[cfg(feature = "server")]
pub fn save_chunk_data(chunk: &Chunk, map_name: &str) -> Result<(), Box<dyn Error>> {
	// Create directory
	fs::create_dir(&chunk.ref_.resource_dir(map_name, false))?;
	// Serialize
	let raw_string: String = serde_json::to_string(chunk)?;
	// Save to file
	let chunk_dir = chunk.ref_.resource_dir(map_name, false);
	fs::write(chunk_dir.clone() + &ChunkDirComponent::JsonData.file_name(), &raw_string)?;
	// Also save chunk texture if there is one
	if let Some(chunk_texture) = &chunk.texture_opt {
		fs::write(chunk_dir + &ChunkDirComponent::Texture.file_name(), &chunk_texture.raw_data)?;
	}
	Ok(())
}

// Misc
pub fn load_file_with_better_error(path: &str) -> Result<String, Box<dyn Error>> {// With help from ChatGPT because I'm lasy
	match fs::read_to_string(path) {
        Ok(contents) => Ok(contents),
        Err(err) => {
            // Combine the error with the path information
            Err(Box::new(IoError::new(err.kind(), format!("Error reading file '{}': {}", path, err))))
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