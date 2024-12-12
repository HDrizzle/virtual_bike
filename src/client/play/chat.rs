//! Created 2024-3-8, has bevy plugin for chat

use bevy_inspector_egui::{
	bevy_egui::{egui::{Ui, Window}, EguiContexts}// Importing from re-export to prevent conflicting versions of bevy_egui
};
use bevy::prelude::*;
use renet::{DefaultChannel, RenetClient};

use crate::prelude::*;

struct GuiState {
    pub curr_entry: String
}

impl Default for GuiState {
    fn default() -> Self {
        Self {
            curr_entry: String::new()
        }
    }
}

// Systems
fn update_system(
    mut egui_contexts: EguiContexts,
	mut renet_client: ResMut<RenetClient>,
	mut state: Local<GuiState>,
	auth: Res<ClientAuth>,
    msg_log: ResMut<message_log::Log>
) {
    Window::new("Messages/Chat").show(egui_contexts.ctx_mut(), |ui: &mut Ui| {
        // Recieved messages
        for msg in &msg_log.messages {
            ui.label(msg.to_string());
        }
        // Send message
        ui.text_edit_singleline(&mut state.curr_entry);
        if ui.button("Send").clicked() {
            renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&RenetRequest::Chat(auth.clone(), state.curr_entry.clone())).unwrap());
            state.curr_entry = String::new();
        }
    });
}

pub struct ChatGuiPlugin;

impl Plugin for ChatGuiPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(message_log::Log::new());
        app.add_systems(Update, update_system);
    }
}