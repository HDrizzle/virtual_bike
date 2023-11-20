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
fn update_system(
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
        AppState::WaitingOnInit => {egui::Window::new("Network").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
            ui.label("Waiting for server...");
        });},
        AppState::Initiated => {egui::Window::new("Network").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
            if ui.button("Sign out").clicked() {
                *app_state = AppState::Login;
                *input_state = SigninEntryState::default();
            }
        });}
    };
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