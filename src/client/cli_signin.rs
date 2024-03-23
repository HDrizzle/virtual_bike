//! Created 2023-11-24
//! Replacing the egui signin window

use std::{time::Duration, thread, sync::{Arc, Mutex}, mem, net, io::Write};
use bevy_renet::renet::DefaultChannel;
//use extras::prompt;

use crate::prelude::*;
use super::{SigninEntryState, play, cache, asset_client::{AssetLoaderManager, self}, load_static_data, load_vehicle_models};

const BACKSPACE: char = 8u8 as char;

fn loading_screen(go: Arc<Mutex<bool>>) {
	println!("Loading");
	let chars: [&str; 4] = ["-", "\\", "|", "/"];
	let delay = Duration::from_millis(250);
	let mut i: usize = 1;
	loop {
		// Print
		std::io::stdout().flush().unwrap();
		print!("{}\r", chars[i]);
		// Update i
		i += 1;
		if i >= chars.len() {
			i = 0;
		}
		// Check mutex to see of this should end
		if !*go.lock().unwrap() {
			break;
		}
		// Wait
		thread::sleep(delay);
	}
}

pub fn get_play_init_info() -> Option<play::InitInfo> {
	// Load settings
	let settings = resource_interface::load_client_settings().unwrap();
	// Create client/transport
	let entry: SigninEntryState = {
		#[cfg(feature = "client_default_signin")]
		{
			println!("Feature `client_default_signin` enabled, using default sign in");
			SigninEntryState::default()
		}
		#[cfg(not(feature = "client_default_signin"))]
		{
			println!("Enter information to connect to the game server");
			SigninEntryState {
				name: prompt("Username"),
				psswd: prompt("Password"),
				ip: prompt("IPv4 addr"),
				port: prompt("Port")
			}
		}
	};
	let mut network_init_info: play::NetworkInitInfo = entry.build_network_init_info();
	// Start loading animation
	let go_mutex = Arc::new(Mutex::new(true));
	let go_mutex_clone = go_mutex.clone();
	let thread_handle = thread::spawn(move || loading_screen(go_mutex_clone));
	// Asset client
	let mut asset_client = {
		let renet_addr = network_init_info.renet_server_addr;
		AssetLoaderManager::new(&settings, net::SocketAddr::new(renet_addr.ip(), renet_addr.port() + 1))
	};
	let out: play::InitInfo = {
		let static_data = load_static_data(&mut network_init_info).unwrap();
		//let vehicle_static_models = load_vehicle_models(&mut asset_client, &static_data, &settings, &network_init_info).unwrap();
		play::InitInfo {
			network: network_init_info,
			static_data,
			settings,
			asset_client,
			//vehicle_static_models
		}
	};
	// Done
	*go_mutex.lock().unwrap() = false;
	thread_handle.join().unwrap();
	Some(out)
}