//! For loading assets/big files from the server's HTTP asset server

use std::{sync::{Arc, Mutex}, time::{Instant, Duration}, net::SocketAddr, thread, mem};
use bevy::{prelude::*, render::settings};
use reqwest;

use crate::prelude::*;

#[derive(Resource)]
pub struct AssetLoaderManager {
	loaders: Vec<HttpAssetLoader>,
	retry_time: Duration,
	pub server_addr: SocketAddr
}

impl AssetLoaderManager {
	pub fn new(settings: &super::Settings, server_addr: SocketAddr) -> Self {
		Self {
			loaders: Vec::new(),
			retry_time: Duration::from_secs_f32(settings.request_retry_time),
			server_addr
		}
	}
	/// Initiates new request, returns request hash if request was initiated or None if request was a duplicate
	pub fn request(&mut self, request: AssetRequest) -> Option<u64> {
		let dup_result = self.is_request_duplicate(&request);
		if dup_result.0 {
			return None;
		}
		let loader = HttpAssetLoader::new(
			self.server_addr,
			self.retry_time,
			request
		);
		self.loaders.push(loader);
		// Done
		Some(dup_result.1)
	}
	/// Checks if the request is already in action by using its hash, returns ((bool) whether request is a duplicate, (u64) hash)
	fn is_request_duplicate(&self, request: &AssetRequest) -> (bool, u64) {
		let hash = calculate_hash(&request.to_owned());
		for loader in &self.loaders {
			if loader.request_hash == hash {
				return (true, hash);
			}
		}
		(false, hash)
	}
	/// Checks on all of the loaders and returns Vec of results for the ones that have finished
	pub fn update(&mut self) -> Vec<Result<AssetResponse, String>> {
		let mut indices_to_delete = Vec::<usize>::new();
		let mut out = Vec::<Result<AssetResponse, String>>::new();
		for (i, loader) in self.loaders.iter_mut().enumerate() {
			match loader.check() {
				Some(res) => {
					out.push(res);
					indices_to_delete.push(i);
				},
				None => {}
			}
		}
		// Delete
		for i in indices_to_delete.iter().rev() {// Important to reverse indices when deleting
			self.loaders.remove(*i);
		}
		// Done
		out
	}
}

pub struct HttpAssetLoader {
	latest_request: Arc<Mutex<Instant>>,
	retry_time: Duration,
	result_opt: Arc<Mutex<Option<Result<AssetResponse, String>>>>,
	request_handle: thread::JoinHandle<()>,
	pub request_hash: u64
}

impl HttpAssetLoader {
	pub fn new(server_addr: SocketAddr, retry_time: Duration, request: AssetRequest) -> Self {
		let request_hash = calculate_hash(&request);
		// Latest request
		let latest_request = Arc::new(Mutex::new(Instant::now()));
		let latest_request_clone = latest_request.clone();
		// Retry time
		let retry_time_clone = retry_time.clone();
		// Result option
		let result_opt: Arc<Mutex<Option<Result<AssetResponse, String>>>> = Arc::new(Mutex::new(None));
		let result_opt_clone = result_opt.clone();
		// Start thread
		let request_handle = thread::spawn(
			move || {
				Self::main_loop(server_addr, request, result_opt_clone);
			}
		);
		// Done
		Self {
			latest_request,
			retry_time,
			result_opt,
			request_handle,
			request_hash
		}
	}
	pub fn check(&mut self) -> Option<Result<AssetResponse, String>> {
		mem::replace(&mut *self.result_opt.lock().unwrap(), None)// I love this function
	}
	fn main_loop(
		server_addr: SocketAddr,
		request: AssetRequest,
		result_opt: Arc<Mutex<Option<Result<AssetResponse, String>>>>
	) {
		let url: String = format!("http://{:?}{}", server_addr, request.request_path());
		let response_result = to_string_err(reqwest::blocking::get(url));
		*result_opt.lock().unwrap() = Some(match response_result {
			Ok(res) => {
				let raw_data: Vec<u8> = res.bytes().unwrap().into_iter().collect();
				match bincode::deserialize::<AssetResponse>(&raw_data) {
					Ok(response) => Ok(response),
					Err(e) => Err(format!("Bincode deserialization error: {}", e))
				}
			},
			Err(e) => Err(e)
		});
	}
}