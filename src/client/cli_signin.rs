//! Created 2023-11-24
//! Replacing the egui signin window

use std::{time::Duration, thread, sync::{Arc, Mutex}, mem, net};
use bevy_renet::renet::DefaultChannel;
//use extras::prompt;

use crate::prelude::*;
use super::{SigninEntryState, play, cache, asset_client::{AssetLoaderManager, self}};

const BACKSPACE: char = 8u8 as char;

fn loading_screen(go: Arc<Mutex<bool>>) {
	let chars: [&str; 4] = ["-", "\\", "|", "/"];
	let delay = Duration::from_millis(250);
	print!("-");
	let mut i: usize = 1;
	loop {
		// Print
		print!("{}", BACKSPACE);
		print!("{}", chars[i]);
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
			print!("Feature `client_default_signin` enabled, using default sign in");
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
	// Asset client
	let mut asset_client = {
		let renet_addr = network_init_info.renet_server_addr;
		AssetLoaderManager::new(&settings, net::SocketAddr::new(renet_addr.ip(), renet_addr.port() + 1))
	};
	// Start loading animation
	let go_mutex = Arc::new(Mutex::new(true));
	let go_mutex_clone = go_mutex.clone();
	let thread_handle = thread::spawn(move || loading_screen(go_mutex_clone));
	// Wait for server to respond
	let mut static_data_opt: Option<StaticData> = None;
	let mut requested_static_data = false;
	let mut requested_vehicle_models = false;
	let out: play::InitInfo = loop {
		// Renet example in README: https://crates.io/crates/renet
		let dt = Duration::from_millis(10);
		thread::sleep(dt);
		network_init_info.renet_client.update(dt);
		network_init_info.renet_transport.update(dt, &mut network_init_info.renet_client).unwrap();
		// Handle messages
		if !network_init_info.renet_client.is_disconnected() {
			// Send static data request if haven't already
			if !requested_static_data {
				network_init_info.renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&RenetRequest::Init).unwrap());
				//network_init_info.renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::VehicleRawGltfData("test-bike".to_owned())).unwrap());
				requested_static_data = true;
			}
			// Check if static data has been recieved
			let mut done = false;// Can't break from inside while loop, use this flag instead
			while let Some(message) = network_init_info.renet_client.receive_message(DefaultChannel::ReliableOrdered) {
				let res = bincode::deserialize::<RenetResponse>(&message).unwrap();
				match res {
					RenetResponse::Err(e) => panic!("Server sent following error message: {}", e),
					res => match &static_data_opt {
						Some(_) => {}
						None => {
							if let RenetResponse::InitState(static_data) = res {// Static data has been recieved from server
								println!("\nRecieved static data");
								// Request vehicle models
								if settings.cache && !requested_vehicle_models {
									for v_type in static_data.vehicle_models_to_load(network_init_info.renet_server_addr.ip()) {
										asset_client.request(AssetRequest::VehicleRawGltfData(v_type));
									}
									requested_vehicle_models = true;
									println!("Requested vehicle models");
								}
								// Save static data
								static_data_opt = Some(static_data);
							}
						}
					}
				}
			}
			// Loop over asset client results
			if let Some(static_data) = &static_data_opt {
				for asset_response_result in asset_client.update() {
					match asset_response_result {
						Ok(asset_response) => if let AssetResponse::VehicleRawGltfData(v_static_model) = asset_response {
							println!("Recieved model for vehicle type {}", &v_static_model.name());
							//cache::save_static_vehicle_model(network_init_info.addr, &v_type, data).unwrap();
							v_static_model.save(network_init_info.renet_server_addr.ip()).unwrap();
						},
						Err(e) => println!("Error from asset client: {}", e)
					}
				}
				if static_data.vehicle_models_to_load(network_init_info.renet_server_addr.ip()).len() == 0 || !settings.cache {// Whether all vehicle models have been loaded
					done = true;
					println!("No more vehicle models to load, loop should exit now");
				}
			}
			if done {
				let static_data_opt_owned: Option<StaticData> = mem::replace(&mut static_data_opt, None);
				match static_data_opt_owned {
					Some(static_data) => {
						break play::InitInfo {
							network: network_init_info,
							static_data,
							settings,
							asset_client
						};
					},
					None => panic!("The program is in a state which it is not supposed to be able to be in AAAAAHHHHHHH")
				}
				
			}
		}
		network_init_info.renet_transport.send_packets(&mut network_init_info.renet_client);// Don't unwrap this, it will cause errors and it turns out that just ignoring them works
	};
	// Done
	*go_mutex.lock().unwrap() = false;
	thread_handle.join().unwrap();
	Some(out)
}