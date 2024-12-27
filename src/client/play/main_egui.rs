//! Main egui for navigation and path bound lateral forces

use bevy_inspector_egui::bevy_egui::{egui::{Ui, Window}, EguiContexts, EguiSet};
use bevy::prelude::*;

use crate::prelude::*;

/// Currently a placeholder, will probably use
#[derive(Default)]
#[allow(unused)]
struct ForcesGuiState;

// Systems
/// Draws egui each frame, info to display:
/// * Power
/// * Diagram showing the lateral forces when on path
/// * (optional) route
/// * (optional) upcomming intersection
fn forces_update_system(
	mut egui_contexts: EguiContexts,
	auth: Res<ClientAuth>,
	world_state: Res<WorldSend>
) {
	Window::new("Body forces").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
		match world_state.vehicles.get(&auth.name) {
			Some(v) => match &v.path_forces_opt {
				Some(forces) => forces.render_to_egui(ui),
				None => {}
			}
			None => {}
		}
	});
}

pub struct ForcesGuiPlugin;

impl Plugin for ForcesGuiPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Update, forces_update_system);
	}
}

fn nav_update_system(
	mut egui_contexts: EguiContexts,
	static_data: Res<StaticData>,
	auth: Res<ClientAuth>,
	world_state: Res<WorldSend>
) {
	Window::new("Navigation").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
		match world_state.vehicles.get(&auth.name) {
			Some(v) => match &v.route_opt {// Check if vehicle has a route query
				Some(og_route_ref) => {
					// Check if route query works in the context of the client's static data
					match static_data.map.path_set.generic.routes.get_item_tuple(&og_route_ref.to_query()) {
						Some((route_ref, _route)) => ui.label(format!("Route: {}", route_ref.to_string())),
						None => ui.label(format!("Route query ({:?}) appeears to be invalid", og_route_ref))
					};
				},
				None => {
					ui.label("No route");
				}
			}
			None => {}
		}
	});
}

pub struct NavGuiPlugin;

impl Plugin for NavGuiPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Update, nav_update_system.after(EguiSet::InitContexts));
	}
}