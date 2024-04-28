//! At 2023-11-21 I moved all currently existing bevy-based code into mod 'play'
//! This module contains the bevy-based module `play` and sign-in screen logic using egui and eframe
//! Template for using `eframe` copied from https://github.com/appcove/egui.info/blob/master/examples/egui-101-basic/src/main.rsc

use std::{net, thread, time::{Duration, SystemTime}};
use bevy::ecs::system::Resource;
use bevy_renet::renet::{DefaultChannel, RenetClient, transport::{NetcodeClientTransport, ClientAuthentication}};
use local_ip_address::local_ip;
use serde::{Serialize, Deserialize};

use crate::prelude::*;

// Mods
#[cfg(feature = "egui_signin")]
mod egui_signin;
#[cfg(not(feature = "egui_signin"))]
mod cli_signin;
pub mod play;
pub mod hardware_controller;
pub mod cache;
pub mod asset_client;
use asset_client::AssetLoaderManager;

#[derive(Serialize, Deserialize, Resource)]
pub struct Settings {
	pub cache: bool,
	pub request_retry_time: Float
}

#[derive(Debug)]
struct SigninEntryState {
	pub name: String,
	pub psswd: String,
	pub ip: String,
	pub port: String
}

impl SigninEntryState {
	pub fn valid(&self) -> bool {
		true// TODO
	}
	pub fn build_network_init_info(&self) -> play::NetworkInitInfo {
		assert!(self.valid());
		// Setup the transport layer TODO: use entry_state
		let server_port: u16 = self.port.parse::<u16>().expect("Unable to parse server port");//resource_interface::load_port().expect("Unable to load and parse port.txt");
		let server_ip: net::IpAddr = self.ip.parse::<net::IpAddr>().expect("Unable to parse server IP address");
		let server_addr: net::SocketAddr = net::SocketAddr::new(server_ip, server_port);
		let client_socket: net::UdpSocket = net::UdpSocket::bind(net::SocketAddr::new(// see https://doc.rust-lang.org/nightly/std/net/struct.UdpSocket.html#method.connect
			if server_ip == net::Ipv4Addr::new(127, 0, 0, 1) {// If the server is on localhost, use localhost address
				net::IpAddr::V4(net::Ipv4Addr::new(127, 0, 0, 1))
			}
			else {// Otherwise, use LAN address
				local_ip().expect("Could not get this machine's local ip address, this is required because the server's address is not localhost (127.0.0.1)")
			},
			0u16
		)).unwrap();
		// ClientAuthentication
		let client_auth = ClientAuthentication::Unsecure {
			protocol_id: 0,
			client_id: calculate_hash(&self.name),// Hash username for `client_id`, https://stackoverflow.com/questions/29573605/how-do-i-use-stdhashhash
			server_addr,
			user_data: None
		};
		let current_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
		let transport = NetcodeClientTransport::new(current_time, client_auth, client_socket).unwrap();
		let client = RenetClient::new(server::connection_config());
		//client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::Init).unwrap());
		play::NetworkInitInfo {
			renet_transport: transport,
			renet_client: client,
			auth: ClientAuth{name: self.name.clone(), psswd: self.psswd.clone()},
			renet_server_addr: server_addr
		}
	}
}

impl Default for SigninEntryState {
	fn default() -> Self {
		Self {
			name: "admin".to_string(),
			psswd: "1234".to_string(),// Don't look, very private
			ip: "127.0.0.1".to_string(),
			port: "62062".to_string()
		}
	}
}

/// Loads the static data from the renet server
fn load_static_data(network_init_info: &mut play::NetworkInitInfo) -> Result<StaticData, String> {
	// Send static data request
	network_init_info.renet_client.send_message(DefaultChannel::ReliableUnordered, bincode::serialize(&RenetRequest::Init).unwrap());
	loop {
		// Renet example in README: https://crates.io/crates/renet
		let dt = Duration::from_millis(10);
		thread::sleep(dt);
		network_init_info.renet_client.update(dt);
		network_init_info.renet_transport.update(dt, &mut network_init_info.renet_client).unwrap();
		// Handle messages
		if !network_init_info.renet_client.is_disconnected() {
			// Check if static data has been recieved
			while let Some(message) = network_init_info.renet_client.receive_message(DefaultChannel::ReliableOrdered) {
				let res = bincode::deserialize::<RenetResponse>(&message).unwrap();
				match res {
					RenetResponse::InitState(static_data) => {
						return Ok(static_data);
					},
					RenetResponse::Err(e) => {return Err(format!("Server sent following error message: {}", e));},
					_ => {}
				}
			}
		}
		let _ = network_init_info.renet_transport.send_packets(&mut network_init_info.renet_client);// Don't unwrap this, it will cause errors and it turns out that just ignoring them works
		thread::sleep(Duration::from_millis(100));// Don't waste CPU
	};
}

/// Gets all of the static vehicle binary GLB 3D models from the asset server
#[deprecated]// Bevy web asset means this is no longer needed
#[allow(unused)]
fn load_vehicle_models(asset_client: &mut AssetLoaderManager, static_data: &StaticData, settings: &Settings, network_init_info: &play::NetworkInitInfo) -> Result<Vec<VehicleStaticModel>, String> {
	// Request vehicle models
	let mut static_vehicles_requested: usize = 0;
	for v_type in static_data.all_vehicle_types() {
		asset_client.request(AssetRequest::VehicleRawGltfData(v_type));
		static_vehicles_requested += 1;
	}
	println!("Requested vehicle models");
	// Wait for responses
	let mut out = Vec::<VehicleStaticModel>::new();
	while static_vehicles_requested > 0 {
		for asset_response_result in asset_client.update() {
			match asset_response_result {
				Ok(asset_response) => if let AssetResponse::VehicleRawGltfData(v_static_model) = asset_response {
					static_vehicles_requested -= 1;
					println!("Recieved model for vehicle type {}", &v_static_model.name());
					if settings.cache {
						v_static_model.save(network_init_info.renet_server_addr).unwrap();
					}
				},
				Err(e) => {return Err(format!("Error from asset client: {}", e));}
			}
		}
		thread::sleep(Duration::from_millis(100));// Don't waste CPU
	}
	// Done
	Ok(out)
}

fn get_play_init_info() -> Option<play::InitInfo> {
	#[cfg(feature = "egui_signin")]
	return egui_signin::get_play_init_info();
	#[cfg(not(feature = "egui_signin"))]
	cli_signin::get_play_init_info()
}

pub fn start() {
	loop {
		let play_init = match get_play_init_info() {
			Some(x) => x,
			None => break
		};
		// Open bevy main app
		play::start(play_init);
		println!("End of loop");
		break;// for testing, TODO: remove
	}
}