//! For dealing with cached assets for the client
//! Caches corresponding to each server will be stored in CACHE_DIR/<IP addr>:<port #>/

pub const CACHE_DIR: &str = "assets/client_cache/";

use std::{fs, path, net::SocketAddr};
#[allow(unused)]
use crate::prelude::*;
use bevy::prelude::*;

pub trait CacheableBevyAsset: Sized {
	/// Ex: "static_vehicles/"
	const CACHE_SUB_DIR: &'static str;
	/// Ex: `Scene` for GLTF / GLB
	type BevyAssetType: Asset;
	/// Ex: "test-bike"
	fn name(&self) -> String;
	/// Ex: "test-bike/model.glb"
	fn cache_path(name: &str) -> String;
	fn write_to_file(&self, file: &mut std::fs::File) -> Result<(), String>;
	fn save(&self, server_addr: SocketAddr) -> Result<(), String> {
		let path = Self::get_path(&self.name(), server_addr);
		let mut path_buf = path::Path::new(&path).to_path_buf();
		assert!(path_buf.pop());// removes filename https://doc.rust-lang.org/stable/std/path/struct.PathBuf.html#method.pop
		fs::create_dir_all(path_buf).unwrap();//.split("/").next().unwrap_or("<Error: path could not be split with forward slash>"));
		self.write_to_file(&mut fs::File::create(path).unwrap())
	}
	fn get_path(name: &str, server_addr: SocketAddr) -> String {
		format!("{}{}{}", get_or_create_cache_dir(server_addr), Self::CACHE_SUB_DIR, Self::cache_path(name))
	}
	fn load_to_bevy(name: &str, server_addr: SocketAddr, asset_server: &AssetServer) -> Result<Handle<Self::BevyAssetType>, String> {
		let path_raw = Self::get_path(name, server_addr);
		dbg!(&path_raw);
		match path_raw.strip_prefix("assets/") {
			Some(path) => Ok(asset_server.load(path.to_owned())),
			None => Err(format!("Unable to strip \"assets/\" off the beginning of \"{}\" so it can be compatible with bevy's asset server", &path_raw))
		}
	}
	fn is_already_cached(&self, server_addr: SocketAddr) -> bool {
		path::Path::new(&Self::get_path(&self.name(), server_addr)).exists()
	}
}

fn get_or_create_cache_dir(addr: SocketAddr) -> String {
	format!("{}{:?}/", CACHE_DIR, addr)
}

/*
#[deprecated]
pub fn get_static_vehicle_model_path(addr: SocketAddr, type_: &str) -> String {
	format!("{}static_vehicles/{}/{}", get_or_create_cache_dir(addr), type_, VEHICLE_MODEL_FILE_NAME)
}

#[deprecated]
pub fn save_static_vehicle_model(addr: SocketAddr, type_: &str, data: Vec<u8>) -> Result<(), String> {
	let path: String = get_static_vehicle_model_path(addr, type_);
	// This will create the dir if it doesn't exist. If it does it will return an Err(), which can be ignored. TODO: crash if the error is something other than `dir_` already existing
	fs::create_dir_all(path.strip_suffix(VEHICLE_MODEL_FILE_NAME).unwrap_or("<Error: vehicle model path does not contain correct ending>")).unwrap();//.split("/").next().unwrap_or("<Error: path could not be split with forward slash>"));
	to_string_err_with_message(fs::write(&path, data), &format!("Error writing static vehicle glb model file to {}", path))
}

#[deprecated]
pub fn is_vehicle_model_cached(addr: SocketAddr, type_: &str) -> bool {
	path::Path::new(&get_static_vehicle_model_path(addr, type_)).exists()
}*/