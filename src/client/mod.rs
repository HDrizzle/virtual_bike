/* At 2023-11-21 I moved all currently existing bevy-based code into mod 'play'
This module contains the bevy-based module `play` and sign-in screen logic using egui and eframe
Template for using `eframe` copied from https://github.com/appcove/egui.info/blob/master/examples/egui-101-basic/src/main.rsc
*/
use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, time::{SystemTime, Duration}, rc::Rc, cell::RefCell, mem, ops::DerefMut};
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

enum UiState {
	Signin(SigninEntryState),// Wait for user to type in valid info
	Waiting {// Waiting for server to respond with StaticData, loading screen
		renet_transport: NetcodeClientTransport,// Trying https://doc.rust-lang.org/std/mem/fn.replace.html
		renet_client: RenetClient,
		auth: ClientAuth
	}
}

impl Default for UiState {
	fn default() -> Self {
		Self::Signin(SigninEntryState::default())
	}
}

struct App {
	state: UiState,
	quit: bool,// Whether the main loop should break and be done
	static_data: Option<StaticData>,
	horcrux_opt: Rc<RefCell<Option<Self>>>// For returning data after window is closed, Harry Potter reference
}

impl App {
	fn new(cc: &eframe::CreationContext<'_>, horcrux_opt: Rc<RefCell<Option<Self>>>) -> Self {
		// Customize egui here with cc.egui_ctx.set_fonts and cc.egui_ctx.set_visuals.
		// Restore app state using cc.storage (requires the "persistence" feature).
		// Use the cc.gl (a glow::Context) to create graphics shaders and buffers that you can use
		// for e.g. egui::PaintCallback.
		Self {
			state: UiState::default(),
			quit: false,
			static_data: None,
			horcrux_opt
		}
	}
	pub fn build_play_init_info(app: &mut Self) -> Option<play::InitInfo> {// Self-canabolistic
		match &mut app.state {
			UiState::Signin(..) => {return None;},
			UiState::Waiting{..} => {}
		}
		// This code will only run if app.state == Waiting, takes ownership of app.state
		let app_state_owned: UiState = mem::replace(&mut app.state, UiState::default());
		match app_state_owned {
			UiState::Signin(..) => panic!("This should not be possible"),
			UiState::Waiting { renet_transport, renet_client, auth } => match &app.static_data {
				Some(static_data) => Some(play::InitInfo {
					renet_transport: renet_transport,
					renet_client: renet_client,
					static_data: static_data.clone(),// Possibly mem::replace()
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
		let mut new_state: Option<UiState> = None;
		let mut new_static_data: Option<StaticData> = None;
		let mut create_new_horcrux: bool = false;
		match &mut self.state {
			UiState::Signin(entry_state) => {
				egui::CentralPanel::default().show(ctx, |ui: &mut Ui| {
					ui.horizontal(|ui: &mut Ui| {
						ui.label("Username: ");
						ui.text_edit_singleline(&mut entry_state.name);
					});
					ui.horizontal(|ui: &mut Ui| {
						ui.label("Password: ");
						ui.text_edit_singleline(&mut entry_state.psswd);
					});
					ui.horizontal(|ui: &mut Ui| {
						ui.label("IPv4 addr: ");
						ui.text_edit_singleline(&mut entry_state.ip);
					});
					ui.horizontal(|ui: &mut Ui| {
						ui.label("Port: ");
						ui.text_edit_singleline(&mut entry_state.port);
					});
					if entry_state.valid() {
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
							new_state = Some(UiState::Waiting {
								renet_transport: transport,
								renet_client: client,
								auth: ClientAuth{name: entry_state.name.clone(), psswd: entry_state.psswd.clone()}
							});
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
							new_static_data = Some(static_data);
							create_new_horcrux = true;
							frame.close();
						}
					}
					// TODO: update transport like in example here: https://crates.io/crates/renet
				});
			}
		};
		match new_state {
			Some(state) => self.state = state,
			None => {}
		}
		match new_static_data {
			Some(static_data) => self.static_data = Some(static_data),
			None => {}
		}
		let temp: Self = self.move_out();
		if create_new_horcrux {
			match *self.horcrux_opt.borrow_mut() {
				Some(ref mut horcrux) => {*horcrux = temp},
				None => panic!("App doesn't have a horcrux")
			}
		}		
   }
}

impl Default for App {
	fn default() -> Self {
		Self {
			state: UiState::default(),
			quit: false,
			static_data: None,
			horcrux_opt: Rc::new(RefCell::new(None))
		}
	}
}

pub fn start() {
	loop {
		// Open sign-in window
		let play_init: play::InitInfo = {
			let native_options = eframe::NativeOptions::default();
			let mut horcrux_opt = Rc::new(RefCell::new(Some(App::default())));
			let mut horcrux_opt_move_into_closure = horcrux_opt.clone();
			eframe::run_native(APP_NAME, native_options, Box::new(|cc| Box::new(App::new(cc, horcrux_opt_move_into_closure)))).unwrap();
			// Window has now been closed
			let temp: play::InitInfo = match horcrux_opt.borrow_mut().deref_mut() {
				Some(horcrux) => match App::build_play_init_info(horcrux) {
					Some(play_init) => play_init,
					None => break
				},
				None => break
			}; temp// Compiler said to do this I won't question it
		};
		// Open bevy main app
		play::start(play_init);
	}
}