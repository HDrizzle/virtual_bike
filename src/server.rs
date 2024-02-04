// Created 2023-7-29, some things copied from `web_server`
// see README on https://github.com/lucaspoffo/renet
use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, time::{SystemTime, Duration, Instant, UNIX_EPOCH}, thread, sync::{mpsc, Arc}, mem};
use renet::{RenetServer, ServerEvent, ConnectionConfig, SendType, ChannelConfig, transport::{ServerAuthentication, ServerConfig}, transport::NetcodeServerTransport, DefaultChannel};
use serde::{Serialize, Deserialize};
use bincode;
use local_ip_address::local_ip;
use rouille;// Simple HTTP Server
//use rocket::{State, Rocket};
//use futures::executor::block_on;

use crate::{prelude::*, world::async_messages, resource_interface};

pub const BIG_DATA_SEND_UNRELIABLE: ChannelConfig = ChannelConfig {
	channel_id: 3,// first 3 are the unreliable and reliable(un)ordered channels
    max_memory_usage_bytes: 500_000_000,// Arbitrary
    send_type: SendType::ReliableUnordered{resend_time: Duration::from_secs(5)}
};

#[derive(Serialize, Deserialize)]
pub enum RenetRequest {// All possible requests
	Init,
	VehicleRawGltfData(String),
	ClientUpdate(ClientUpdate),
	Chunk {
		chunk_ref: ChunkRef,
		with_texture: bool
	},
	TogglePlaying,
	RecoverVehicleFromFlip(ClientAuth),
	NewUser {
		name: String,
		psswd: String
	}
}

#[derive(Serialize, Deserialize)]
pub enum RenetResponse {// All possible responses
	InitState(StaticData),
	VehicleRawGltfData(VehicleStaticModel),// Username, file contents
	WorldState(WorldSend),
	Chunk(Chunk),
	Err(String)
}

#[cfg(feature = "server")]
struct NetworkRuntimeManager {
	pub server: RenetServer,
	pub addr: SocketAddr,
	pub transport: NetcodeServerTransport,
	pub static_data: StaticData
}

#[cfg(feature = "server")]
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
								self.server.broadcast_message(DefaultChannel::Unreliable, bincode::serialize(&RenetResponse::WorldState(world_send)).unwrap());
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
					let decoded_result: Result<RenetRequest, Box<bincode::ErrorKind>> = bincode::deserialize(&message[..]);
					match decoded_result {
						Ok(decoded) => match decoded {
							RenetRequest::Init => {
								println!("Received init request");
								self.server.send_message(*client_id, DefaultChannel::ReliableOrdered, bincode::serialize(&RenetResponse::InitState(self.static_data.clone())).expect("Unable to serialize static data with bincode"));
							},
							RenetRequest::VehicleRawGltfData(name) => {
								println!("Recieved vehicle model request for: {}", &name);
								let load_result: Result<Vec<u8>, String> = resource_interface::load_static_vehicle_gltf(&name);
								self.server.send_message(*client_id, BIG_DATA_SEND_UNRELIABLE.channel_id, bincode::serialize(&match load_result {
									Ok(data) => RenetResponse::VehicleRawGltfData(VehicleStaticModel::new(name.clone(), data)),
									Err(e) => RenetResponse::Err(e)
								}).expect("Unable to serialize vehicle raw GLTF file data with bincode"));
							},
							RenetRequest::ClientUpdate(update) => {
								// TODO: authenticate client
								tx.send(async_messages::ToWorld::ClientUpdate(update)).expect("Unable to send client update to world");
							},
							RenetRequest::Chunk{chunk_ref, with_texture} => {
								//println!("Recieved chunk request: {:?}", &chunk_ref);
								//self.server.send_message(*client_id, BIG_DATA_SEND_UNRELIABLE.channel_id, bincode::serialize(&Response::Err("Test error sent over BIG_DATA_SEND_UNRELIABLE".to_owned())).unwrap());
								match Chunk::load(&chunk_ref, &self.static_data.map.name) {
									Ok(mut chunk) => {
										chunk.set_texture_opt(with_texture, &self.static_data.map.name).unwrap();
										let res = RenetResponse::Chunk(chunk);
										println!("Sending chunk: {:?}", &chunk_ref);
										self.server.send_message(*client_id, BIG_DATA_SEND_UNRELIABLE.channel_id, bincode::serialize(&res).expect("Unable to serialize chunk with bincode"));
									},
									Err(e) => {
										println!("Error loading chunk {:?}: {:?}", &chunk_ref, e);
										tx.send(async_messages::ToWorld::CreateChunk(chunk_ref)).expect("Unable to send chunk creation request to world");
										//Response::Err(format!("Error loading chunk: {}", e.to_string()))
									}
								}
							},
							RenetRequest::TogglePlaying => {
								// TODO: authenticate client
								tx.send(async_messages::ToWorld::TogglePlaying).expect("Unable to send client update to world");
							},
							RenetRequest::RecoverVehicleFromFlip(auth) => {
								// TODO: authenticate client
								tx.send(async_messages::ToWorld::RecoverVehicleFromFlip(auth)).expect("Unable to send message to world");
							},
							RenetRequest::NewUser{name, psswd} => {
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

#[cfg(feature = "server")]
pub struct WorldServer {
	world: World,
	net_manager_opt: Option<NetworkRuntimeManager>,
	asset_server_thread_handle: thread::JoinHandle<()>
}

#[cfg(feature = "server")]
impl WorldServer {
	pub fn init(world_name: &str, localhost: bool) -> Self {
		// Load world
		let world = World::load(world_name).expect(&format!("Failed to load world \"{}\"", world_name));
		let static_data = world.build_static_data();
		// Init renet server, mostly copied from https://crates.io/crates/renet
		let server = RenetServer::new(connection_config());
		let ip_addr: IpAddr = if localhost {
			IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1))
		}
		else {
			local_ip().expect("Could not get this machine's local ip address, to run on localhost/127.0.0.1 pass the flag `-localhost`")
		};
		let renet_port = resource_interface::load_port().expect("Unable to load and parse port.txt");
		let addr = SocketAddr::new(ip_addr, renet_port);
		println!("Renet server listening on {:?}", addr);
		let socket: UdpSocket = UdpSocket::bind(addr).unwrap();
		let server_config = ServerConfig {
			current_time: SystemTime::now().duration_since(UNIX_EPOCH).unwrap(),
			max_clients: 64,
			protocol_id: 0,
			public_addresses: vec![addr],
			authentication: ServerAuthentication::Unsecure
		};
		let current_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
		let transport = NetcodeServerTransport::new(server_config, socket).unwrap();
		// Init HTTP server
		let asset_server = AssetServer::new(
			world_name.to_owned(),
			world.map.generic.name.clone()
		);
		let asset_server_thread_handle = asset_server.start(SocketAddr::new(ip_addr, renet_port + 1));
		// Done
		Self {
			world,
			net_manager_opt: Some(NetworkRuntimeManager {
				server,
				addr,
				transport,
				static_data
			}),
			asset_server_thread_handle
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
				None => panic!("Network Runtime Manager option isn't Some. Has this function been run more than once?")
			};
			(thread::spawn(move || net_manager_owned.main_loop(net_rx, net_tx)), world_rx, world_tx)
		};
		// Main loop
		self.world.main_loop(world_rx, world_tx);
		// Join network thread
		println!("World done, joining network thread");
		network_thread_handle.join().unwrap();
	}
}

pub fn connection_config() -> ConnectionConfig {
	let mut channels_config: Vec<ChannelConfig> = DefaultChannel::config();
	channels_config.push(BIG_DATA_SEND_UNRELIABLE.clone());
	ConnectionConfig {
		available_bytes_per_tick: 60_000,
		server_channels_config: channels_config.clone(),
		client_channels_config: channels_config
	}
}

#[derive(Clone)]
struct AssetServer {// Nothing to do with Bevy
	world_name: String,
	map_name: String
}

impl AssetServer {
	pub fn new(
		world_name: String,
		map_name: String,
	) -> Self {
		Self {
			world_name,
			map_name
		}
	}
	pub fn start(&self, addr: SocketAddr) -> thread::JoinHandle<()> {
		let self_clone_arc = Arc::new(self.clone());
		thread::Builder::new().name(format!("{} HTTP Server", APP_NAME)).spawn(
			move || {
				rouille::start_server(
					addr,
					move |req: &rouille::Request| -> rouille::Response {
						self_clone_arc.request_handler(req)
					}
				);
			}
		).unwrap()
	}
	pub fn request_handler(&self, req: &rouille::Request) -> rouille::Response {
		match req.url().as_str() {
			"/" => rouille::Response::text(format!("{} Asset Server", APP_NAME)),
			"/hello" => rouille::Response::text("Hello World"),
			_ => rouille::Response::empty_404()
		}
	}
}

/*
#[get("/chunks/<id>")]
fn get_chunk(state: &State<HttpServerStaticState>, id: String) -> String {
	format!("Request for chunk: \"{}\"", id)
}

#[get("/")]
fn get_base() -> String {
	format!("{} HTTP Server", APP_NAME)
}*/