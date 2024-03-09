//! Created 2023-7-29
//! see README on https://github.com/lucaspoffo/renet
use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, time::{SystemTime, Duration, Instant, UNIX_EPOCH}, thread, sync::{mpsc, Arc}, mem};
use renet::{RenetServer, ServerEvent, ConnectionConfig, SendType, ChannelConfig, transport::{ServerAuthentication, ServerConfig}, transport::NetcodeServerTransport, DefaultChannel};
use serde::{Serialize, Deserialize};
use bincode;
use local_ip_address::local_ip;
#[cfg(feature = "server")]
use rouille;// Simple HTTP Server
use bytes::Bytes;

use crate::{prelude::*, world::async_messages, resource_interface};

// mods
pub mod message_log;
use message_log::{Message, MessageEnum, Log};

// CONSTS
pub const BIG_DATA_SEND_UNRELIABLE: ChannelConfig = ChannelConfig {
	channel_id: 3,// first 3 are the unreliable and reliable(un)ordered channels
    max_memory_usage_bytes: 500_000_000,// Arbitrary
    send_type: SendType::ReliableUnordered{resend_time: Duration::from_secs(5)}
};

#[derive(Serialize, Deserialize)]
pub enum RenetRequest {// All possible requests
	Init,
	ClientUpdate(ClientUpdate),
	TogglePlaying,
	RecoverVehicleFromFlip(ClientAuth),
	NewUser {
		name: String,
		psswd: String
	},
	Chat(ClientAuth, String)
}

#[derive(Serialize, Deserialize)]
pub enum RenetResponse {// All possible responses
	InitState(StaticData),
	WorldState(WorldSend),
	Err(String),
	Message(Message)
}

/// Not actually serialized and sent to the server, just a way to represent asset HTTP requests
#[cfg(feature = "client")]
#[derive(Hash)]
pub enum AssetRequest {
	VehicleRawGltfData(String),
	Chunk {
		chunk_ref: ChunkRef,
		with_texture: bool
	}
}

#[cfg(feature = "client")]
impl AssetRequest {
	/// For example: "/chunks/100_200?with_texture=0" or "/vehicle_static_models/cool_bike"
	pub fn request_path(&self) -> String {
		match self {
			Self::VehicleRawGltfData(static_name) => format!("/vehicle_static_models/{}", static_name),
			Self::Chunk{chunk_ref, with_texture} => format!(
				"/chunks/{}?with_texture={}",
				chunk_ref.resource_dir_name(),
				match with_texture {
					true => "1",
					false => "0"
				}
			)
		}
	}
}

#[derive(Serialize, Deserialize)]
pub enum AssetResponse {
	VehicleRawGltfData(VehicleStaticModel),// Username, file contents
	Chunk(Chunk)
}

#[cfg(feature = "server")]
struct NetworkRuntimeManager {
	pub server: RenetServer,
	pub addr: SocketAddr,
	pub transport: NetcodeServerTransport,
	pub static_data: StaticData,
	pub message_log: Log
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
				let mut messages = Vec::<Bytes>::new();
				while let Some(message) = self.server.receive_message(*client_id, DefaultChannel::ReliableOrdered) {
					messages.push(message);
				}
				while let Some(message) = self.server.receive_message(*client_id, DefaultChannel::ReliableUnordered) {
					messages.push(message);
				}
				while let Some(message) = self.server.receive_message(*client_id, DefaultChannel::Unreliable) {
					messages.push(message);
				}
				for message in messages {
					let decoded_result: Result<RenetRequest, Box<bincode::ErrorKind>> = bincode::deserialize(&message[..]);
					match decoded_result {
						Ok(decoded) => match decoded {
							RenetRequest::Init => {
								println!("Received init request");
								self.server.send_message(*client_id, DefaultChannel::ReliableOrdered, bincode::serialize(&RenetResponse::InitState(self.static_data.clone())).expect("Unable to serialize static data with bincode"));
							},
							RenetRequest::ClientUpdate(update) => {
								// TODO: authenticate client
								tx.send(async_messages::ToWorld::ClientUpdate(update)).expect("Unable to send client update to world");
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
							RenetRequest::Chat(auth, chat) => {
								// TODO: authenticate client
								self.new_message(MessageEnum::UserChat{name: auth.name, chat});
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
				self.new_message(match event {
					ServerEvent::ClientConnected{client_id} => MessageEnum::ClientConnected(client_id.to_string()),// TODO: client name
					ServerEvent::ClientDisconnected{client_id, reason} => MessageEnum::ClientDisconnected(client_id.to_string(), reason.to_string())// TODO: client name
				});
			}
			// Send packets to clients
			self.transport.send_packets(&mut self.server);
		}
	}
	fn new_message(&mut self, msg_enum: MessageEnum) {
		let msg = Message::new(msg_enum);
		self.message_log.add(msg.clone());
		self.server.broadcast_message(DefaultChannel::ReliableOrdered, bincode::serialize(&RenetResponse::Message(msg)).unwrap());
	}
}

#[cfg(feature = "server")]
pub struct WorldServer {
	world_name: String,
	world: World,
	net_manager_opt: Option<NetworkRuntimeManager>,
	asset_server_addr: SocketAddr
	//asset_server_thread_handle: thread::JoinHandle<()>
}

#[cfg(feature = "server")]
impl WorldServer {
	pub fn init(world_name: String, localhost: bool) -> Self {
		// Load world
		let world = World::load(&world_name).expect(&format!("Failed to load world \"{}\"", world_name));
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
			protocol_id: 0,// TODO: hash program version like what the renet docs says, so different versions can't connect
			public_addresses: vec![addr],
			authentication: ServerAuthentication::Unsecure
		};
		let current_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
		let transport = NetcodeServerTransport::new(server_config, socket).unwrap();
		// Done
		Self {
			world_name,
			world,
			net_manager_opt: Some(NetworkRuntimeManager {
				server,
				addr,
				transport,
				static_data,
				message_log: Log::new()
			}),
			asset_server_addr: SocketAddr::new(ip_addr, renet_port + 1)
			//asset_server_thread_handle
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
		let (network_thread_handle, world_rx, world_tx, asset_server_tx) = {
			let (net_tx, world_rx) = mpsc::channel::<async_messages::ToWorld>();// To thread
			let asset_server_tx = net_tx.clone();
			let (world_tx, net_rx) = mpsc::channel::<async_messages::FromWorld>();// From thread
			let mut net_manager_owned = match mem::replace(&mut self.net_manager_opt, None) {
				Some(net_manager) => net_manager,
				None => panic!("Network Runtime Manager option isn't Some. Has this function been run more than once?")
			};
			(thread::spawn(move || net_manager_owned.main_loop(net_rx, net_tx)), world_rx, world_tx, asset_server_tx)
		};
		// Init HTTP server
		let asset_server = AssetServer::new(
			self.world_name.to_owned(),
			self.world.map.generic.name.clone(),
			asset_server_tx
		);
		let _asset_server_thread_handle = asset_server.start(self.asset_server_addr);
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

#[cfg(feature = "server")]
fn response_code_and_message(code: u16, message: String) -> rouille::Response {
	rouille::Response {
		status_code: code,
		headers: Vec::new(),
		data: rouille::ResponseBody::from_string(message),
		upgrade: None
	}
}

#[cfg(feature = "server")]
#[derive(Clone)]
struct AssetServer {// Nothing to do with Bevy
	world_name: String,
	map_name: String,
	tx: mpsc::Sender<async_messages::ToWorld>
}

#[cfg(feature = "server")]
impl AssetServer {
	pub fn new(
		world_name: String,
		map_name: String,
		tx: mpsc::Sender<async_messages::ToWorld>
	) -> Self {
		Self {
			world_name,
			map_name,
			tx
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
		let binding = req.url();
		let url_parts: Vec<&str> = binding.strip_prefix("/").expect("url should start with \"/\"").split("/").collect();
		//dbg!(&url_parts);
		match url_parts[0] {
			"" => rouille::Response::text(format!("{} Asset Server", APP_NAME)),
			"hello" => rouille::Response::text("Hello World"),
			"chunks" => match url_parts.len() >= 2 {
				true => {
					let with_texture: bool = true;// TODO
					match ChunkRef::from_resource_dir_name(url_parts[1]) {
						Ok(chunk_ref) => {
							match Chunk::load(&chunk_ref, &self.map_name, with_texture) {
								Ok(chunk) => rouille::Response::from_data("application/octet-stream", bincode::serialize(&AssetResponse::Chunk(chunk)).expect("Unable to serialize chunk")),
								Err(e) => {
									self.tx.send(async_messages::ToWorld::CreateChunk(chunk_ref)).expect("Unable to send chunk creation request to world");
									response_code_and_message(400, format!("Error loading chunk: {}, request to create chunk was sent to the world server", e))
								}
							}
						},
						Err(e) => response_code_and_message(400, e)
					}
				},
				false => {
					rouille::Response::text("Chunks")
				}
			},
			"vehicle_static_models" => match url_parts.len() >= 2 {
				true => {
					match resource_interface::load_static_vehicle_gltf(url_parts[1].strip_suffix(".glb").unwrap_or(url_parts[1])) {// Will work with or wthout extension
						Ok(raw_file) => rouille::Response::from_data("application/octet-stream", raw_file),//bincode::serialize(&AssetResponse::VehicleRawGltfData(VehicleStaticModel::new(url_parts[1].to_owned(), raw_file))).expect("Unable to serialize vehicle static model")),
						Err(e) => response_code_and_message(404, format!("Error loading vehicle static model: {}", e))
					}
				},
				false => {
					rouille::Response::text("Vehicle static models")
				}
			},
			_ => rouille::Response::empty_404()
		}
	}
}