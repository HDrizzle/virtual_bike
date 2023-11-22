// For handling the Renet connection with the server
use std::net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr};
use bevy::prelude::*;
use bevy_inspector_egui::{bevy_egui::EguiContexts, {egui, egui::Ui}};

use super::*;
use crate::prelude::*;

// Resources
#[derive(Resource)]
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
}

impl Default for SigninEntryState {
	fn default() -> Self {
		Self {
			name: "".to_string(),
			psswd: "".to_string(),
			ip: "".to_string(),
			port: "".to_string()
		}
	}
}

// Systems
fn egui_system(
	mut commands: Commands,
	mut egui_contexts: EguiContexts,
	mut renet_client: ResMut<RenetClient>,
	mut input_state: Local<SigninEntryState>,
	mut app_state: ResMut<AppState>
) {
	match *app_state {
		AppState::Login => {egui::Window::new("Network").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
			ui.horizontal(|ui: &mut Ui| {
				ui.label("Username: ");
				ui.text_edit_singleline(&mut input_state.name);
			});
			ui.horizontal(|ui: &mut Ui| {
				ui.label("Password: ");
				ui.text_edit_singleline(&mut input_state.psswd);
			});
			ui.horizontal(|ui: &mut Ui| {
				ui.label("IPv4 addr: ");
				ui.text_edit_singleline(&mut input_state.ip);
			});
			ui.horizontal(|ui: &mut Ui| {
				ui.label("Port: ");
				ui.text_edit_singleline(&mut input_state.port);
			});
			if input_state.valid() {
				if ui.button("Sign in").clicked() {
					// Setup the transport layer
					let server_port = resource_interface::load_port().expect("Unable to load and parse port.txt");
					let server_addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), server_port);
					let client_socket = UdpSocket::bind(SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 0u16)).unwrap();// see https://doc.rust-lang.org/nightly/std/net/struct.UdpSocket.html#method.connect
					let client_auth = ClientAuthentication::Unsecure {
						protocol_id: 0,
						client_id: 0,
						server_addr: server_addr,
						user_data: None
					};
					let current_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
					let transport = NetcodeClientTransport::new(current_time, client_auth, client_socket).unwrap();
					commands.insert_resource(transport);
					let mut client = RenetClient::new(ConnectionConfig::default());
					println!("Sent init request");
					client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::Init).unwrap());
					commands.insert_resource(client);
					// Change app state
					*app_state = AppState::WaitingOnInit;
				}
			}
		});},
		AppState::WaitingOnInit => {
			// GUI
			egui::Window::new("Network").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
				ui.label("Waiting for server...");
			});
		},
		AppState::Initiated => {egui::Window::new("Network").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
			if ui.button("Sign out").clicked() {
				*app_state = AppState::Login;
				*input_state = SigninEntryState::default();
			}
		});}
	};
	// Incomming messages
	let mut most_recent_world_state: Option<WorldSend> = None;
	let mut messages = Vec::<Vec<u8>>::new();
	while let Some(message) = renet_client.receive_message(DefaultChannel::ReliableOrdered) {
		messages.push(message.to_vec());
	}
	while let Some(message) = renet_client.receive_message(DefaultChannel::Unreliable) {
		messages.push(message.to_vec());
	}
	for message in messages {
		let res = bincode::deserialize::<Response>(&message).unwrap();
		match res {
			Response::InitState(mut static_data) => {
				if app_state == &AppState::WaitingOnInit {// Init
					println!("Received static data");
					// Test, TODO fix
					//renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::Chunk(ChunkRef{position: [100, 100]})).unwrap());
					// Init rapier
					// (HashMap<String, RigidBodyHandle>, Vec<(String, Wheel)>)
					let (body_handles, owner_wheel_pairs) = static_data.init_bevy_rapier_context(&mut rapier_context);
					*v_body_handles = VehicleBodyHandles(body_handles);
					// Save wheels
					for pair in owner_wheel_pairs.iter() {
						commands.spawn((pair.1.clone(), UsernameComponent(pair.0.clone())));
					}
					// Save map body handle
					*map_body_handle_opt = Some(static_data.map.body_handle_opt.expect("Could not get map body handle"));
					// Save map as resource
					commands.insert_resource(static_data.map);
					// Done
					*app_state = AppState::Initiated;
				}
			},
			Response::WorldState(world_send) => {
				most_recent_world_state = Some(world_send);// In case multiple come through, only use the most recent one
			},
			Response::Chunk(chunk) => {
				//println!("Recieved chunk");//, but doing nothing else because debug reasons");
				requested_chunks.remove(&chunk.ref_);
				match &mut map_res_opt {
					Some(map_res) => {
						map_res.insert_chunk_client(chunk, &mut rapier_context, &mut commands, &mut meshes, &mut materials, &asset_server);
					},
					None => panic!("There is no Map resource, but Chunks are being recieved")
				}
			},
			Response::Err(err_string) => {
				panic!("Server sent following error message: {}", err_string);
			}
		}
	}
}

// Plugins
pub struct CustomRenetPlugin;

impl Plugin for CustomRenetPlugin {
	fn build(&self, app: &mut App) {
		app.add_plugins(RenetClientPlugin);
		app.add_plugins(NetcodeClientPlugin);
		app.add_systems(Update, update_system);
	}
}