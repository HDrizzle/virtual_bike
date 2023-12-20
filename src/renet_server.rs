// Created 2023-7-29, some things copied from `web_server`
// see README on https://github.com/lucaspoffo/renet
use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, time::{SystemTime, Duration}, thread, sync::{mpsc, Arc, Mutex}, mem};
use renet::{RenetServer, ServerEvent, ConnectionConfig, transport::{ServerAuthentication, ServerConfig}, transport::NetcodeServerTransport, DefaultChannel};
use serde::{Serialize, Deserialize};
use bincode;
use local_ip_address::local_ip;

use crate::{prelude::*, world::async_messages, resource_interface};

#[derive(Serialize, Deserialize)]
pub enum Request {// All possible requests
	Init,
	VehicleRawGltfData(String),
	ClientUpdate(ClientUpdate),
	Chunk(ChunkRef),
	TogglePlaying,
	RecoverVehicleFromFlip(ClientAuth),
	NewUser {
		name: String,
		psswd: String
	}
}

#[derive(Serialize, Deserialize)]
pub enum Response {// All possible responses
	InitState(StaticData),
	VehicleRawGltfData(String, Vec<u8>),// Username, file contents
	WorldState(WorldSend),
	Chunk(Chunk),
	Err(String)
}

struct NetworkRuntimeManager {
	pub server: RenetServer,
	pub addr: SocketAddr,
	pub transport: NetcodeServerTransport,
	pub static_data: StaticData
}

impl NetworkRuntimeManager {
	pub fn main_loop(&mut self, rx: mpsc::Receiver<async_messages::FromWorld>, tx: mpsc::Sender<async_messages::ToWorld>) {
		// Main loop
		loop {
			thread::sleep(Duration::from_millis(100));// Placeholder to prevent the computer from freezing
			// Receive new messages and update clients, copied from github readme
			let delta_time = Duration::from_millis(100);
			self.server.update(delta_time);
			self.transport.update(delta_time, &mut self.server).unwrap();
			// Receive messages from world simulation thread
			loop {
				match rx.try_recv() {// https://doc.rust-lang.org/stable/std/sync/mpsc/struct.Receiver.html
					Ok(update) => {
						match update {
							async_messages::FromWorld::State(world_send) => {
								// Broadcast state to all clients
								self.server.broadcast_message(DefaultChannel::Unreliable, bincode::serialize(&Response::WorldState(world_send)).unwrap());
							}
							async_messages::FromWorld::Error(e) => panic!("Received the following error from world: {}", e)
						}
					},
					Err(error) => {
						match error {
							mpsc::TryRecvError::Empty => break,
							mpsc::TryRecvError::Disconnected => panic!("World receive channel has been disconnected")
						}
					}
				}
			}
			// Receive messages from server
			for client_id in self.server.clients_id().iter() {
				// The enum DefaultChannel describe the channels used by the default configuration
				while let Some(message) = self.server.receive_message(*client_id, DefaultChannel::ReliableOrdered) {
					let decoded_result: Result<Request, Box<bincode::ErrorKind>> = bincode::deserialize(&message[..]);
					match decoded_result {
						Ok(decoded) => match decoded {
							Request::Init => {
								println!("Received init request");
								self.server.send_message(*client_id, DefaultChannel::ReliableOrdered, bincode::serialize(&Response::InitState(self.static_data.clone())).expect("Unable to serialize static data with bincode"));
							},
							Request::VehicleRawGltfData(name) => {
								println!("Recieved vehicle model request for: {}", &name);
								let load_result: Result<Vec<u8>, String> = resource_interface::load_static_vehicle_gltf(&name);
								self.server.send_message(*client_id, DefaultChannel::ReliableOrdered, bincode::serialize(&match load_result {
									Ok(data) => Response::VehicleRawGltfData(name.clone(), data),
									Err(e) => Response::Err(e)
								}).expect("Unable to serialize vehicle raw GLTF file data with bincode"));
							},
							Request::ClientUpdate(update) => {
								// TODO: authenticate client
								tx.send(async_messages::ToWorld::ClientUpdate(update)).expect("Unable to send client update to world");
							},
							Request::Chunk(chunk_ref) => {
								// TODO: change to Reliable
								//println!("Recieved chunk request: {:?}", &chunk_ref);
								match Chunk::load(&chunk_ref, &self.static_data.map.name) {
									Ok(chunk) => {
										let res = Response::Chunk(chunk);
										self.server.send_message(*client_id, DefaultChannel::ReliableOrdered, bincode::serialize(&res).expect("Unable to serialize chunk with bincode"));
									},
									Err(e) => {
										tx.send(async_messages::ToWorld::CreateChunk(chunk_ref)).expect("Unable to send chunk creation request to world");
										//Response::Err(format!("Error loading chunk: {}", e.to_string()))
									}
								}
							},
							Request::TogglePlaying => {
								// TODO: authenticate client
								tx.send(async_messages::ToWorld::TogglePlaying).expect("Unable to send client update to world");
							},
							Request::RecoverVehicleFromFlip(auth) => {
								// TODO: authenticate client
								tx.send(async_messages::ToWorld::RecoverVehicleFromFlip(auth)).expect("Unable to send message to world");
							},
							Request::NewUser{name, psswd} => {
								todo!();// TODO
							}
						},
						Err(e) => {
							let err_string = format!("Error decoding request with bincode: {}", e.to_string());
							self.server.send_message(*client_id, DefaultChannel::ReliableOrdered, err_string.as_bytes().to_vec());
						}
					}
				}
			}
			// Check for client connections/disconnections, TODO more usefull
			while let Some(event) = self.server.get_event() {
				match event {
					ServerEvent::ClientConnected { client_id } => {
						println!("Client {client_id} connected");
					}
					ServerEvent::ClientDisconnected { client_id, reason } => {
						println!("Client {client_id} disconnected: {reason}");
					}
				}
			}
			// Send packets to clients
			self.transport.send_packets(&mut self.server);
		}
	}
}

pub struct WorldServer {
	world: World,
	net_manager_opt: Option<NetworkRuntimeManager>
}

impl WorldServer {
	pub fn init(world_name: &str, localhost: bool) -> Self {
		// Load world
		let world = World::load(world_name).expect(&format!("Failed to load world \"{}\"", world_name));
		let static_data = world.build_static_data();
		// Init renet server, mostly copied from https://crates.io/crates/renet
		let server = RenetServer::new(ConnectionConfig::default());
		let ip_addr: IpAddr = if localhost {
			IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1))
		}
		else {
			local_ip().expect("Could not get this machine's local ip address, to run on localhost/127.0.0.1 pass the flag `-localhost`")
		};
		let addr = SocketAddr::new(ip_addr, resource_interface::load_port().expect("Unable to load and parse port.txt"));
		println!("Server listening on {:?}", addr);
		let socket: UdpSocket = UdpSocket::bind(addr).unwrap();
		let server_config = ServerConfig {
			max_clients: 64,
			protocol_id: 0,
			public_addr: addr,
			authentication: ServerAuthentication::Unsecure
		};
		let current_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
		let transport = NetcodeServerTransport::new(current_time, server_config, socket).unwrap();
		Self {
			world,
			net_manager_opt: Some(NetworkRuntimeManager {
				server,
				addr,
				transport,
				static_data
			})
		}
	}
	pub fn main_loop(&mut self) {
		// Start world simulation
		/*
		let (_world_thread_handle, world_tx, world_rx) = {
			let (tx, rx_remote) = mpsc::channel::<async_messages::ToWorld>();// To thread
			let (tx_remote, rx) = mpsc::channel::<async_messages::FromWorld>();// From thread
			let world_copy = self.world.clone();
			(thread::spawn(move || ((*world_copy).lock().unwrap()).main_loop(rx_remote, tx_remote)), tx, rx)
		};*/
		let (network_thread_handle, world_rx, world_tx) = {
			let (net_tx, world_rx) = mpsc::channel::<async_messages::ToWorld>();// To thread
			let (world_tx, net_rx) = mpsc::channel::<async_messages::FromWorld>();// From thread
			let mut net_manager_owned = match mem::replace(&mut self.net_manager_opt, None) {
				Some(net_manager) => net_manager,
				None => panic!("Network Runtime Manager option in Some. Has this function been run more than once?")
			};
			(thread::spawn(move || net_manager_owned.main_loop(net_rx, net_tx)), world_rx, world_tx)
		};
		// Main loop
		self.world.main_loop(world_rx, world_tx);
		// Join network thread
		println!("World done, joining network thread");
		network_thread_handle.join();
	}
}