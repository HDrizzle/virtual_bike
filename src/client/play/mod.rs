/* Bevy-based client app
Rapier plugin example (2D but similar enough): https://www.youtube.com/watch?v=GwlZ5EPu8l0
Rapier inspector (old version because current version failed build): https://docs.rs/crate/bevy-inspector-egui-rapier/latest
TODO: sometime when this gets more developed use states: https://bevy-cheatbook.github.io/programming/states.html
Bevy 3D rendering simple example: https://bevyengine.org/examples/3D%20Rendering/3d-scene/

Major change 2023-11-21: this module will only be used when the game is signed-in and being played
*/

use std::{net::{SocketAddr, UdpSocket, IpAddr, Ipv4Addr}, time::{SystemTime, Duration}, collections::HashMap};
use bevy::{prelude::*, winit::WinitSettings, input::{keyboard::KeyboardInput, ButtonState}, render::mesh::PrimitiveTopology};
use renet::transport::{ServerConfig, ClientAuthentication, NetcodeClientTransport};
use bevy_renet::{renet::*, transport::NetcodeClientPlugin};
use bevy_renet::*;
use bevy_rapier3d::{plugin::RapierContext, prelude::Real};
use rapier3d::dynamics::RigidBodyHandle;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_inspector_egui_rapier::InspectableRapierPlugin;
use nalgebra::geometry::{Isometry, UnitQuaternion};
use bincode;

use crate::{
	prelude::*,
	resource_interface,
	renet_server::{Request, Response}
};

use super::{hardware_controller::HardwareControllerPlugin, network::CustomRenetPlugin};

// Mods
mod chunk_manager;
use chunk_manager::{ChunkManagerPlugin, RenderDistance, RequestedChunks};
mod gui;
use gui::GuiPlugin;

// Custom resources
#[derive(Resource)]
pub struct VehicleBodyHandles(pub HashMap<String, RigidBodyHandle>);

#[derive(Resource)]
pub struct InitInfo {// Provided by the sign-in window, DOES NOT CHANGE
	pub renet_transport: NetcodeClientTransport,
	pub renet_client: RenetClient,
	pub static_data: StaticData,
	pub auth: ClientAuth
}

// Components
#[derive(Component)]
pub struct CameraComponent;

#[derive(Component)]
struct UsernameComponent(String);

// Systems
/*fn startup_test(mut client: ResMut<RenetClient>) {
	println!("Sent init request");
	client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::Init).unwrap());
}*/

fn vehicle_input_key_event_system(// Only for manual vehicle control should be behind the `debug_vehicle_input` feature
	keys: Res<Input<KeyCode>>,
	mut key_events: EventReader<KeyboardInput>,
	mut renet_client: ResMut<RenetClient>,
	mut auth: ResMut<ClientAuth>,
) {
	// Process user input (ONLY TEMPORARY)
	// Key state polling
	let input = {
		// Detect motion
		// Translation
		let mut power_forward = 0.0;
		let power_magnitude = 20.0;
		// Y
		if keys.pressed(KeyCode::W) {
			power_forward += power_magnitude;
		}
		if keys.pressed(KeyCode::S) {
			power_forward -= power_magnitude;
		}
		// Rotation/Steering
		let mut steering = 0.0;
		if keys.pressed(KeyCode::A) {
			steering += 1.0;
		}
		if keys.pressed(KeyCode::D) {
			steering -= 1.0;
		}
		// Create input
		InputData {
			steering: steering,
			speed: power_forward,
			power: power_forward,// TODO: fix
			brake: 0.0
		}
	};
	let client_update = ClientUpdate {
		auth: auth.into_inner().clone(),
		input
	};
	renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::ClientUpdate(client_update)).unwrap());
}

fn misc_key_event_system(
	keys: Res<Input<KeyCode>>,
	mut key_events: EventReader<KeyboardInput>,
	mut renet_client: ResMut<RenetClient>,
	mut auth_res: ResMut<ClientAuth>,
) {
	let auth = auth_res.into_inner().clone();
	// Key events
	for ev in key_events.iter() {
		match ev.state {
			ButtonState::Pressed => {
				match ev.key_code {
					Some(key_code) => match key_code {
						KeyCode::P => renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::TogglePlaying).unwrap()),
						KeyCode::E => renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::RecoverVehicleFromFlip(auth.clone())).unwrap()),
						_ => {}
					}
					None => {}
				}
			}
			ButtonState::Released => {
				// Unused
			}
		}
	}
}

fn update_system(
	mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
	mut map_res_opt: Option<ResMut<Map>>,
	asset_server: Res<AssetServer>,
	mut renet_client: ResMut<RenetClient>,
	mut rapier_context_res: ResMut<RapierContext>,
	mut camera_query: Query<(&mut CameraComponent, &mut Transform, &Projection)>,
	mut wheel_query: Query<(&mut Wheel, &UsernameComponent)>,
	mut v_body_handles_res: ResMut<VehicleBodyHandles>,
	mut map_body_handle_opt: Local<Option<RigidBodyHandle>>,
	mut requested_chunks: ResMut<RequestedChunks>,
	mut render_distance: ResMut<RenderDistance>,
	init_info: Res<InitInfo>
) {
	// How to move a camera: https://bevy-cheatbook.github.io/cookbook/pan-orbit-camera.html
	let mut rapier_context = rapier_context_res.into_inner();
	let mut v_body_handles = v_body_handles_res.into_inner();
	// Incomming messages
	let mut most_recent_world_state: Option<WorldSend> = None;
	let mut messages = Vec::<Vec<u8>>::new();
	while let Some(message) = renet_client.receive_message(DefaultChannel::ReliableOrdered) {
		messages.push(message.to_vec());
	}
	while let Some(message) = renet_client.receive_message(DefaultChannel::Unreliable) {
		messages.push(message.to_vec());
	}
	for message in messages {
		let res = bincode::deserialize::<Response>(&message).unwrap();
		match res {
			Response::InitState(mut static_data) => {
				// TODO: move this to super
				println!("Received static data");
				// Test, TODO fix
				//renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::Chunk(ChunkRef{position: [100, 100]})).unwrap());
				// Init rapier
				// (HashMap<String, RigidBodyHandle>, Vec<(String, Wheel)>)
				let (body_handles, owner_wheel_pairs) = static_data.init_bevy_rapier_context(&mut rapier_context);
				*v_body_handles = VehicleBodyHandles(body_handles);
				// Save wheels
				for pair in owner_wheel_pairs.iter() {
					commands.spawn((pair.1.clone(), UsernameComponent(pair.0.clone())));
				}
				// Save map body handle
				*map_body_handle_opt = Some(static_data.map.body_handle_opt.expect("Could not get map body handle"));
				// Save map as resource
				commands.insert_resource(static_data.map);
			},
			Response::WorldState(world_send) => {
				most_recent_world_state = Some(world_send);// In case multiple come through, only use the most recent one
			},
			Response::Chunk(chunk) => {
				//println!("Recieved chunk");//, but doing nothing else because debug reasons");
				requested_chunks.remove(&chunk.ref_);
				match &mut map_res_opt {
					Some(map_res) => {
						map_res.insert_chunk_client(chunk, &mut rapier_context, &mut commands, &mut meshes, &mut materials, &asset_server);
					},
					None => panic!("There is no Map resource, but Chunks are being recieved")
				}
			},
			Response::Err(err_string) => {
				panic!("Server sent following error message: {}", err_string);
			}
		}
	}
	// This will be like the "main loop"
	if let Some(world_state) = most_recent_world_state {
		// Update all bevy things, first get vect of wheels
		/*let mut wheel_owners = Vec::<(&String, &mut Wheel)>::new();
		for (mut w, owner) in wheel_query.iter_mut() {
			wheel_owners.push((&owner.0, &mut w));
		}*/
		let mut wheel_owners = Vec::<(&String, &mut Wheel)>::new();
		for mut owner in wheel_query.iter_mut() {
			let w = &mut owner.1;
			wheel_owners.push((&w.0, owner.0.into_inner()));
		}
		let map_body_handle = map_body_handle_opt.expect("map_body_handle_opt is None, but AppState == Initiated");
		world_state.update_bevy_rapier_context(&mut rapier_context, &v_body_handles, wheel_owners, Some(map_body_handle));// TODO: fix: replacing `Some(map_body_handle)` with `None` makes the vehicle body update properly
		// Update camera position
		let main_vehicle = world_state.vehicles.get(&init_info.auth.name).expect(&format!("Unable to get vehicle for signed-in username: \"{}\"", &init_info.auth.name));
		render_distance.set_position(p3_to_p2(matrix_to_opoint(main_vehicle.body_state.position.translation.vector.clone())));
		for (_, mut transform, projection) in camera_query.iter_mut() {
			// Comment this out and add manual_camera_control() for debugging
			#[cfg(not(feature = "debug_camera_control"))]
			main_vehicle.update_bevy_camera_transform(&mut transform);
		}
		rapier_context.propagate_modified_body_positions_to_colliders();// Just what I needed https://docs.rs/bevy_rapier3d/latest/bevy_rapier3d/plugin/struct.RapierContext.html#method.propagate_modified_body_positions_to_colliders
	}
}

fn manual_camera_control(
	keys: Res<Input<KeyCode>>,
	mut camera_query: Query<(&mut CameraComponent, &mut Transform, &Projection)>
) {
	for (_, mut transform, projection) in camera_query.iter_mut() {
		// Detect motion
		// Translation
		let mut velocity = Vec3::ZERO;
		// Z
		if keys.pressed(KeyCode::T) {
			velocity.z += 1.0;
		}
		if keys.pressed(KeyCode::G) {
			velocity.z -= 1.0;
		}
		// X
		if keys.pressed(KeyCode::F) {
			velocity.x += 1.0;
		}
		if keys.pressed(KeyCode::H) {
			velocity.x -= 1.0;
		}
		// Y
		if keys.pressed(KeyCode::Space) {
			velocity.y += 1.0;
		}
		if keys.pressed(KeyCode::ShiftLeft) {
			velocity.y -= 1.0;
		}
		let mut vel_trans = Transform::from_translation(velocity);
		vel_trans.rotate_y(transform.rotation.to_euler(EulerRot::YXZ).0);// TODO: fix
		transform.translation += vel_trans.translation * 5.0;// Very sloppy, TODO: fix
		// Rotation
		let mut horiz_rot = 0.0;
		if keys.pressed(KeyCode::Left) {
			horiz_rot += 1.0;
		}
		if keys.pressed(KeyCode::Right) {
			horiz_rot -= 1.0;
		}
		let mut vert_rot = 0.0;
		if keys.pressed(KeyCode::Up) {
			vert_rot += 1.0;
		}
		if keys.pressed(KeyCode::Down) {
			vert_rot -= 1.0;
		}
		let vel_quat_horiz = Quat::from_rotation_y(horiz_rot / 60.0);
		transform.rotation = transform.rotation * vel_quat_horiz;
		let vel_quat_vert = Quat::from_rotation_x(vert_rot / 60.0);
		transform.rotation = transform.rotation * vel_quat_vert;
	}
}

fn render_test(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
	// Copied from https://bevyengine.org/examples/3D%20Rendering/3d-scene/
	// Camera
	commands.spawn((
		Camera3dBundle {
			transform: Transform::from_xyz(50.0, 10.0, 0.0).looking_at(Vec3::new(50., 0., 50.), Vec3::Y),
			..default()
		},
		CameraComponent
	));
	commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(50.0, 50.0, 50.0),
        ..default()
    });
}

struct MainClientPlugin;

impl Plugin for MainClientPlugin {
	fn build(&self, app: &mut App) {
		//app.add_systems(Update, manual_input);
		app.add_systems(Update, misc_key_event_system);
		#[cfg(feature = "debug_vehicle_input")]
		app.add_systems(Update, vehicle_input_key_event_system);
		#[cfg(feature = "debug_camera_control")]
		app.add_systems(Update, manual_camera_control);
		// App state
		/*app.insert_resource(ClientAuth{// TODO: sign-in screen
			name: "admin".to_owned(),
			psswd: "1234".to_owned()
		});*/
		app.insert_resource(VehicleBodyHandles(HashMap::<String, RigidBodyHandle>::new()));
		app.add_systems(Update, update_system);
		// bevy_rapier3d
		app.insert_resource(bevy_rapier3d::plugin::RapierContext::default());
		app.add_plugins(bevy_rapier3d::render::RapierDebugRenderPlugin::default());
		//app.add_plugin(InspectableRapierPlugin);
		app.add_plugins(WorldInspectorPlugin::new());
		// Rendering
		app.insert_resource(ClearColor(Color::rgb(0.7, 0.7, 0.7)));// https://bevy-cheatbook.github.io/window/clear-color.html
		app.add_systems(Startup, render_test);
	}
}

pub fn start(init_info: InitInfo) {
	// TODO: use `init_info`
	let mut app = App::new();
	app.add_plugins((
		DefaultPlugins,
		CustomRenetPlugin,
		ChunkManagerPlugin,
		GuiPlugin,
		HardwareControllerPlugin,
		MainClientPlugin
	));
	app.insert_resource(init_info);
	println!("Starting Bevy app");
	app.run();
}