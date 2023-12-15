/* For dealing with cached assets for the client
Caches corresponding to each server will be stored in CACHE_DIR/<IP addr>:<port #>/
*/
const CACHE_DIR: &str = "assets/client_cache/";
const VEHICLE_MODEL_FILE_NAME: &str = "model.glb";

use std::{fs, path, net::IpAddr};
use crate::prelude::*;

fn get_or_create_cache_dir(addr: IpAddr) -> String {
    format!("{}{:?}/", CACHE_DIR, addr)
}

pub fn get_static_vehicle_model_path(addr: IpAddr, type_: &str) -> String {
    format!("{}vehicles/{}/{}", get_or_create_cache_dir(addr), type_, VEHICLE_MODEL_FILE_NAME)
}

pub fn save_static_vehicle_model(addr: IpAddr, type_: &str, data: Vec<u8>) -> Result<(), String> {
    let path: String = get_static_vehicle_model_path(addr, type_);
    // This will create the dir if it doesn't exist. If it does it will return an Err(), which can be ignored. TODO: crash if the error is something other than `dir_` already existing
    fs::create_dir_all(path.strip_suffix(VEHICLE_MODEL_FILE_NAME).unwrap_or("<Error: vehicle model path does not contain correct ending>"));//.split("/").next().unwrap_or("<Error: path could not be split with forward slash>"));
    to_string_err_with_message(fs::write(&path, data), &format!("Error writing static vehicle glb model file to {}", path))
}

pub fn is_vehicle_model_cached(addr: IpAddr, type_: &str) -> bool {
    path::Path::new(&get_static_vehicle_model_path(addr, type_)).exists()
}