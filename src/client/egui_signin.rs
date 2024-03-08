/* Created 2023-11-24
Code in this file is currently unused, but I spent a lot of time on it and it may be usefull in the future.
It turns out that `winit` which is used by both egui and bevy only allows one window to be opened per process other wise weird things will happen.
I decided to just make the signin a CLI wich is really easy and still achieves the original goal of not having states in the bevy app.
*/

use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, time::{SystemTime, Duration}, rc::Rc, cell::RefCell, mem, ops::DerefMut, thread};
use bevy_renet::renet::{RenetClient, DefaultChannel, ConnectionConfig, transport::{NetcodeClientTransport, ClientAuthentication}};
use bevy_inspector_egui::egui::Ui;
use eframe::egui;

use crate::{prelude::*, renet_server::{Request, Response}};
use super::{SigninEntryState, play};

enum UiState {
	Signin(SigninEntryState),// Wait for user to type in valid info
	Waiting(play::NetworkInitInfo)
}
/*
impl UiState {
	pub fn build_play_init_info(state: &mut Self, static_data: StaticData) -> Option<play::InitInfo> {
		// This code will only run if app.state == Waiting, takes ownership of app.state
		let app_state_owned: UiState = mem::replace(state, UiState::default());
		match app_state_owned {
			UiState::Signin(..) => panic!("This should not be possible"),
			UiState::Waiting { renet_transport, renet_client, auth } => match &state.static_data {
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
}*/

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
		let mut app_state_owned: UiState = mem::replace(&mut app.state, UiState::default());
		match app_state_owned {
			UiState::Signin(..) => panic!("This should not be possible"),
			UiState::Waiting { renet_transport, renet_client, auth } => match &app.static_data {
				Some(static_data) => Some(play::InitInfo {
                    network: play::NetworkInitInfo {
                        renet_transport: renet_transport,
					    renet_client: renet_client,
					    auth: auth.clone()
                    },                   
					static_data: static_data.clone(),// Possibly mem::replace()
				}),
				None => None
			}
		}
		//UiState::build_play_init_info(&mut app_state_owned)
	}
	pub fn move_out(&mut self) -> Self {
		//mem::replace(self, Self::default())
		// Moves everything needed to `build_play_init_info()` nnto another instance
		let state = mem::replace(&mut self.state, UiState::default());
		let static_data = mem::replace(&mut self.static_data, None);
		// Done
		Self {
			state,
			quit: false,
			static_data,
			horcrux_opt: Rc::new(RefCell::new(None))
		}
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
							println!("Sign-in clicked");
							new_state = Some(UiState::Waiting(entry_state.build_network_init_info()));
						}
					}
				});
			},
			UiState::Waiting{renet_transport, ref mut renet_client, auth} => {
				egui::CentralPanel::default().show(ctx, |ui| {
					// Renet example in README: https://crates.io/crates/renet
					let dt = Duration::from_millis(16);
					renet_client.update(dt);
    				renet_transport.update(dt, renet_client).unwrap();
					// Ui
					ui.label("Waiting for server...");
					// Handle messages
					if !renet_client.is_disconnected() {
						while let Some(message) = renet_client.receive_message(DefaultChannel::ReliableOrdered) {
							let res = bincode::deserialize::<Response>(&message).unwrap();
							if let Response::InitState(static_data) = res {// Static data has been recieved from server, set play init info and close this window
								new_static_data = Some(static_data);
								create_new_horcrux = true;
								println!("Recieved static data, Closing frame");
								frame.close();
							}
						}
					}
					renet_transport.send_packets(renet_client);// Don't unwrap this, it will cause errors and it turns out that just them it works
				});
			}
		};
		match new_state {
			Some(state) => {
				self.state = state;
				println!("New UI state");
			},
			None => {}
		}
		match new_static_data {
			Some(static_data) => self.static_data = Some(static_data),
			None => {}
		}
		if create_new_horcrux {
			let temp: Self = self.move_out();
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

pub fn get_play_init_info() -> Option<play::InitInfo> {
    #[cfg(feature = "client_skip_signin")]
    {
        let entry_state = SigninEntryState::default();
        todo!()
    }
    #[cfg(not(feature = "client_skip_signin"))]
    {
		// Open sign-in window
        let native_options = eframe::NativeOptions::default();
        let mut horcrux_opt = Rc::new(RefCell::new(Some(App::default())));
        let mut horcrux_opt_move_into_closure = horcrux_opt.clone();
        eframe::run_native(APP_NAME, native_options, Box::new(|cc| Box::new(App::new(cc, horcrux_opt_move_into_closure)))).unwrap();
        println!("Sign-in window closed");
        // Window has now been closed
        let temp: Option<play::InitInfo> = match horcrux_opt.borrow_mut().deref_mut() {
            Some(horcrux) => match App::build_play_init_info(horcrux) {
                Some(play_init) => Some(play_init),
                None => None
            },
            None => None
        }; temp// Compiler said to do this I won't question it
    }
}