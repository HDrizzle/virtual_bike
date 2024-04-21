//! Main egui for navigation and path bound lateral forces

use bevy_inspector_egui::{
	bevy_egui::{egui::{Ui, Window}, EguiContexts}// Importing from re-export to prevent conflicting versions of bevy_egui
};
use bevy::prelude::*;
use renet::{DefaultChannel, RenetClient};

use crate::prelude::*;

/// Currently a placeholder, will probably use
#[derive(Default)]
struct GuiState;

// Systems
/// Draws egui each frame, info to display:
/// * Power
/// * Diagram showing the lateral forces when on path
/// * (optional) route
/// * (optional) upcomming intersection
fn update_system(
    mut egui_contexts: EguiContexts,
	mut state: Local<GuiState>,
    auth: Res<ClientAuth>,
    world_state: Res<WorldSend>
) {
    Window::new("Info").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
        ui.label("Hello there");
        match world_state.vehicles.get(&auth.name) {
            Some(v) => match &v.path_forces_opt {
                Some(forces) => forces.render_to_egui(ui),
                None => {}
            }
            None => {}
        }
    });
}

pub struct GuiPlugin;

impl Plugin for GuiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, update_system);
    }
}