// Plugin for all Egui stuff

use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, collections::HashMap};
use bevy::{prelude::*, winit::WinitSettings, input::{keyboard::KeyboardInput, ButtonState}, render::mesh::PrimitiveTopology};
//use renet::transport::{ServerConfig, ClientAuthentication, NetcodeClientTransport};
use bevy_renet::{renet::*, transport::NetcodeClientPlugin};
use bevy_renet::*;
use rapier3d::dynamics::RigidBodyHandle;
use bevy_inspector_egui::{
	quick::WorldInspectorPlugin,
	bevy_egui::{egui, EguiContexts, EguiPlugin}// Importing from re-export to prevent conflicting versions of bevy_egui
};
use nalgebra::{point, geometry::{Isometry, UnitQuaternion}};

use crate::prelude::*;

// Structs/enums


// Resources


// Systems
fn egui_update_system(
	mut contexts: EguiContexts
) {
	/*egui::Window::new("Hello World").show(contexts.ctx_mut(), |ui| {
		ui.label("This is a label in Egui");
	});*/
}

// Plugin
pub struct GuiPlugin;

impl Plugin for GuiPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Update, egui_update_system);
		app.add_plugins(EguiPlugin);
	}
}