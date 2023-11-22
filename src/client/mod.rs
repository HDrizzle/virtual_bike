/* At 2023-11-21 I moved all currently existing bevy-based code into mod 'play'
This module contains the bevy-based module `play` and sign-in screen logic using egui and eframe
Template for using `eframe` copied from https://github.com/appcove/egui.info/blob/master/examples/egui-101-basic/src/main.rsc
*/
use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, time::{SystemTime, Duration}, rc::Rc, cell::RefCell, mem};
use bevy_renet::renet::{RenetClient, DefaultChannel, ConnectionConfig, transport::NetcodeClientTransport};
use bevy_inspector_egui::egui::Ui;

use eframe::egui;
use renet::transport::ClientAuthentication;

use crate::{
	prelude::*,
	resource_interface,
	renet_server::{Request, Response}
};

// Mods
pub mod play;
pub mod hardware_controller;
mod network;

enum UiState {
	Signin,// Wait for user to type in valid info
	Waiting {// Waiting for server to respond with StaticData, loading screen
		renet_transport: NetcodeClientTransport,// Trying https://doc.rust-lang.org/std/mem/fn.replace.html
		renet_client: RenetClient,
		auth: ClientAuth
	}
}

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

struct App {
	entry_state: SigninEntryState,
	state: UiState,
	quit: bool,// Whether the main loop should break and be done
	static_data: Option<StaticData>,
	horcrux_opt: Option<Rc<RefCell<Self>>>// For returning data after window is closed, Harry Potter reference
}

impl App {
	fn new(cc: &eframe::CreationContext<'_>, horcrux: Rc<RefCell<Self>>) -> Self {
		// Customize egui here with cc.egui_ctx.set_fonts and cc.egui_ctx.set_visuals.
		// Restore app state using cc.storage (requires the "persistence" feature).
		// Use the cc.gl (a glow::Context) to create graphics shaders and buffers that you can use
		// for e.g. egui::PaintCallback.
		Self {
			entry_state: SigninEntryState::default(),
			state: UiState::Signin,
			quit: false,
			static_data: None,
			horcrux_opt: Some(horcrux)
		}
	}
	pub fn build_play_init_info(app: &mut Self) -> Option<play::InitInfo> {// Self-canabolistic
		match app.state {
			UiState::Signin => None,
			UiState::Waiting { renet_transport, renet_client, auth } => match app.static_data {
				Some(static_data) => Some(play::InitInfo {
					renet_transport: renet_transport,
					renet_client: renet_client,
					static_data: static_data,// Possibly mem::replace()
					auth: auth.clone()
				}),
				None => None
			}
		}
	}
	pub fn move_out(&mut self) -> Self {
		mem::replace(self, Self::default())
	}
}

impl eframe::App for App {
	fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
		match &self.state {
			UiState::Signin => {
				egui::CentralPanel::default().show(ctx, |ui| {
					ui.horizontal(|ui: &mut Ui| {
						ui.label("Username: ");
						ui.text_edit_singleline(&mut self.entry_state.name);
					});
					ui.horizontal(|ui: &mut Ui| {
						ui.label("Password: ");
						ui.text_edit_singleline(&mut self.entry_state.psswd);
					});
					ui.horizontal(|ui: &mut Ui| {
						ui.label("IPv4 addr: ");
						ui.text_edit_singleline(&mut self.entry_state.ip);
					});
					ui.horizontal(|ui: &mut Ui| {
						ui.label("Port: ");
						ui.text_edit_singleline(&mut self.entry_state.port);
					});
					if self.entry_state.valid() {
						if ui.button("Sign in").clicked() {
							// Setup the transport layer TODO: use input_state
							let server_port = resource_interface::load_port().expect("Unable to load and parse port.txt");
							let server_addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), server_port);
							let client_socket = UdpSocket::bind(SocketAddr::new(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)), 0u16)).unwrap();// see https://doc.rust-lang.org/nightly/std/net/struct.UdpSocket.html#method.connect
							let client_auth = ClientAuthentication::Unsecure {
								protocol_id: 0,
								client_id: 0,
								server_addr,
								user_data: None
							};
							let current_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
							let transport = NetcodeClientTransport::new(current_time, client_auth, client_socket).unwrap();
							let mut client = RenetClient::new(ConnectionConfig::default());
							client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::Init).unwrap());
							self.state = UiState::Waiting {
								renet_transport: transport,
								renet_client: client,
								auth: ClientAuth{name: self.entry_state.name.clone(), psswd: self.entry_state.psswd.clone()}
							};
						}
					}
				});
			},
			UiState::Waiting{renet_transport, renet_client, auth} => {
				egui::CentralPanel::default().show(ctx, |ui| {
					// Ui
					ui.label("Waiting for server...");// TODO: loading wheel
					// Handle messages
					while let Some(message) = renet_client.receive_message(DefaultChannel::ReliableOrdered) {
						let res = bincode::deserialize::<Response>(&message).unwrap();
						if let Response::InitState(static_data) = res {// Static data has been recieved from server, set play init info and close this window
							self.static_data = Some(static_data);
							match self.horcrux_opt {
								Some(mut horcrux) => {*horcrux.get_mut() = self.move_out()},
								None => panic!("App doesn't have a horcrux")
							}
							frame.close();
						}
					}
					// TODO: update transport like in example here: https://crates.io/crates/renet
				});
			}
		}
   }
}

impl Default for App {
	fn default() -> Self {
		Self {
			entry_state: SigninEntryState::default(),
			state: UiState::Signin,
			quit: false,
			static_data: None,
			horcrux_opt: None
		}
	}
}

fn start() {
	loop {
		// Open sign-in window
		let play_init = {
			let native_options = eframe::NativeOptions::default();
			let mut horcrux = Rc::new(RefCell::new(App::default()));
			eframe::run_native(APP_NAME, native_options, Box::new(|cc| Box::new(App::new(cc, horcrux)))).unwrap();
			// Window has now been closed
			match App::build_play_init_info(horcrux.get_mut()) {
				Some(play_init) => play_init,
				None => break
			}
		};
		// Open bevy main app
		play::start(play_init);
	}
}