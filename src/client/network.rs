// For handling the Renet connection with the server
use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, time::{SystemTime, Duration}, collections::HashMap};
use bevy::prelude::*;
use bevy_renet::{RenetClientPlugin, transport::NetcodeClientPlugin};

use crate::prelude::*;

// Resources

// Plugins
pub struct CustomRenetPlugin;

impl Plugin for CustomRenetPlugin {
	fn build(&self, app: &mut App) {
		app.add_plugins(RenetClientPlugin);
        app.add_plugins(NetcodeClientPlugin);
        //app.add_systems(Update, update_system);
	}
}