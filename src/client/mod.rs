/* At 2023-11-21 I moved all currently existing bevy-based code into mod 'play'
This module contains the bevy-based module `play` and sign-in screen logic using egui and eframe
Template for using `eframe` copied from https://github.com/appcove/egui.info/blob/master/examples/egui-101-basic/src/main.rsc
*/
use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, time::SystemTime, thread};
use bevy_renet::renet::{RenetClient, ConnectionConfig, transport::{NetcodeClientTransport, ClientAuthentication}};
use local_ip_address::local_ip;

use crate::{
	prelude::*,
	resource_interface
};

// Mods
#[cfg(feature = "egui_signin")]
mod egui_signin;
#[cfg(not(feature = "egui_signin"))]
mod cli_signin;
pub mod play;
pub mod hardware_controller;
mod network;

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
		let server_ip: IpAddr = self.ip.parse::<IpAddr>().expect("Unable to parse server IP address");
		let server_addr: SocketAddr = SocketAddr::new(server_ip, server_port);
		let client_socket: UdpSocket = UdpSocket::bind(SocketAddr::new(// see https://doc.rust-lang.org/nightly/std/net/struct.UdpSocket.html#method.connect
			if server_ip == Ipv4Addr::new(127, 0, 0, 1) {// If the server is on localhost, use localhost address
				IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1))
			}
			else {// Otherwise, use LAN address
				local_ip().expect("Could not get this machine's local ip address, this is required because the server's address is not localhost/127.0.0.1")
			},
			0u16
		)).unwrap();
		let client_auth = ClientAuthentication::Unsecure {
			protocol_id: 0,
			client_id: 0,
			server_addr,
			user_data: None
		};
		let current_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
		let transport = NetcodeClientTransport::new(current_time, client_auth, client_socket).unwrap();
		let client = RenetClient::new(ConnectionConfig::default());
		//client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::Init).unwrap());
		play::NetworkInitInfo {
			renet_transport: transport,
			renet_client: client,
			auth: ClientAuth{name: self.name.clone(), psswd: self.psswd.clone()}
		}
	}
}

impl Default for SigninEntryState {
    fn default() -> Self {
        Self {
            name: "admin".to_string(),
            psswd: "1234".to_string(),// Don't look very private
            ip: "127.0.0.1".to_string(),
            port: "62062".to_string()
        }
    }
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