//!  Bevy-based client app
//! Rapier plugin example (2D but similar enough): https://www.youtube.com/watch?v=GwlZ5EPu8l0
//! Rapier inspector (old version because current version failed build): https://docs.rs/crate/bevy-inspector-egui-rapier/latest
//! TODO: sometime when this gets more developed use states: https://bevy-cheatbook.github.io/programming/states.html
//! Bevy 3D rendering simple example: https://bevyengine.org/examples/3D%20Rendering/3d-scene/
//! 
//! Major change 2023-11-21: this module will only be used when the game is signed-in and being played

use std::{collections::{HashMap, HashSet}, net, f32::consts::PI};
use bevy::{
	prelude::*,
	input::{keyboard::KeyboardInput, ButtonState}
};
use bevy_renet::{RenetClientPlugin, renet::{RenetClient, DefaultChannel}, netcode::NetcodeClientPlugin};
use renet_netcode::NetcodeClientTransport;
use nalgebra::{UnitQuaternion, Isometry};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bincode;
use bevy_web_asset;

use crate::prelude::*;

use super::{hardware_controller::HardwareControllerPlugin, Settings, asset_client::AssetLoaderManager};

// Mods
mod chunk_manager;
use chunk_manager::{ChunkManagerPlugin, RenderDistance, RequestedChunks};
mod skybox;
mod chat;
mod main_egui;

#[derive(Resource)]
pub struct RenetServerAddr(pub net::SocketAddr);

#[derive(Resource, Default, Debug)]
struct StaticVehicleAssetHandles(pub HashMap<String, Handle<Scene>>);// Vehicle type, scene handle

/*#[derive(Resource)]
struct VehicleStaticModelsResource(pub Vec<VehicleStaticModel>);*/

#[derive(Resource)]
enum CameraController {
	Spectator {
		pos: SimpleIso
	},
	Vehicle {
		username: String,
		rel_rot: SimpleRotation,
		radius: Float
	}
}

impl CameraController {
	pub fn vehicle(username: String) -> Self {
		let rel_pos = Self::default_pos_wrt_vehicle();
		Self::Vehicle{username, rel_rot: rel_pos.0, radius: rel_pos.1}
	}
	pub fn toggle(&mut self, vehicles: &HashMap<String, VehicleSend>, username: String) {
		*self = match self {
			Self::Spectator{pos: _} => {
				let rel_pos = Self::default_pos_wrt_vehicle();
				Self::Vehicle{username, rel_rot: rel_pos.0, radius: rel_pos.1}
			},
			Self::Vehicle{username: _, ..} => {
				Self::Spectator{pos: SimpleIso::from_iso(self.get_pos(vehicles))}
			}
		};
	}
	pub fn update(&mut self, dt: Float, trans_input: V3, rot_input: V2) {
		match self {
			Self::Spectator{pos} => {
				let lin_speed: Float = 250.0;// M/s
				let ang_speed: Float = 1.0;// Rad/s
				// Scale translation by speed and dt
				let trans_input_scaled = trans_input * lin_speed * dt;
				// Find yaw angle
				// Rotate translation vector by yaw angle about vertical (Y) axis
				let trans_input_rotated = UnitQuaternion::from_axis_angle(&V3::y_axis(), pos.rotation.yaw + PI/2.0).transform_vector(&trans_input_scaled);
				pos.translation = pos.translation + trans_input_rotated;
				// Delta-yaw
				let delta_yaw = rot_input.x * ang_speed * dt;
				pos.rotation.yaw += delta_yaw;
				// Delta-pitch
				let delta_pitch = rot_input.y * ang_speed * dt;
				pos.rotation.pitch += delta_pitch;
			},
			Self::Vehicle{username: _, rel_rot, radius: _} => {
				let ang_speed: Float = 1.0;// Rad/s
				// Delta-yaw
				let delta_yaw = rot_input.x * ang_speed * dt;
				rel_rot.yaw += delta_yaw;
				// Delta-pitch
				let delta_pitch = rot_input.y * ang_speed * dt;
				rel_rot.pitch += delta_pitch;
			}
		}
	}
	pub fn get_pos(&self, vehicles: &HashMap<String, VehicleSend>) -> Iso {
		match self {
			Self::Spectator{pos} => {
				let og_iso = pos.to_iso();
				Iso {
					rotation: og_iso.rotation,// * UnitQuaternion::from_axis_angle(&V3::y_axis(), PI/2.0),
					translation: og_iso.translation
				}//add_isometries(&Iso::rotation(V3::new(PI/2.0, 0.0, 0.0)), &pos.to_iso()),
			}
			Self::Vehicle{username, rel_rot, radius} => match vehicles.get(&*username) {
				Some(v) => {
					let rel_translation = rel_rot.to_quat().transform_vector(&V3::new(0.0, 0.0, *radius));
					add_isometries(&v.body_state.position, &SimpleIso::new(rel_translation, rel_rot.clone()).to_iso())
				},
				None => {
					bevy::log::warn!("Unable to get vehicle with username \"{}\" to calculate camera position", &*username);
					Isometry::identity()
				}
			}
		}
	}
	pub fn default_pos_wrt_vehicle() -> (SimpleRotation, Float) {
		// Create rotation quat
		let rot = SimpleRotation::new(0.0, -0.2);//UnitQuaternion::from_axis_angle(&V3::x_axis(), -0.2);
		// 2nd POV
		//let rel_translation = rot.to_quat().transform_vector(&V3::new(0.0, 0.0, 30.0));
		(rot, 30.0)
	}
}

/*#[derive(Resource, Default)]
struct VehicleBodyStates(pub HashMap<String, BodyStateSerialize>);

impl VehicleBodyStates {
	pub fn update_from_world_state(&mut self, world: &WorldSend) {
		for (name, v_send) in world.vehicles.iter() {
			self.0.insert(name.to_owned(), v_send.body_state.clone());
		}
	}
}*/

pub struct InitInfo {// Provided by the sign-in window
	pub network: NetworkInitInfo,
	pub static_data: StaticData,
	pub settings: Settings,
	pub asset_client: AssetLoaderManager,
	//pub vehicle_static_models: Vec<VehicleStaticModel>
}

impl InitInfo {
	pub fn setup_app(init_info: Self, app: &mut App) {
		app.insert_resource(CameraController::vehicle(init_info.network.auth.name.clone()));
		app.insert_resource(init_info.static_data);
		app.insert_resource(init_info.network.renet_transport);
		app.insert_resource(init_info.network.renet_client);
		app.insert_resource(init_info.network.auth);
		app.insert_resource(RenetServerAddr(init_info.network.renet_server_addr));
		app.add_systems(Startup, Self::setup_system);
		app.insert_resource(WorldSend::default());
		app.insert_resource(init_info.settings);
		app.insert_resource(init_info.asset_client);
		//app.insert_resource(VehicleStaticModelsResource(init_info.vehicle_static_models));
	}
	pub fn setup_system(
		mut commands: Commands,
		mut static_data: ResMut<StaticData>,
		mut meshes: ResMut<Assets<Mesh>>,
		mut materials: ResMut<Assets<StandardMaterial>>,
		asset_server: Res<AssetServer>,
		asset_client: Res<AssetLoaderManager>
		//vehicle_static_models: Res<VehicleStaticModelsResource>
	) {
		// Map init bevy
		static_data.map.init_bevy(&mut commands, meshes.as_mut(), materials.as_mut(), &*asset_server, &asset_client.server_addr);
		// Load static vehicle gltf model files. This is just to have the asset handles for each vehicle type, not to display anything
		// https://bevy-cheatbook.github.io/3d/gltf.html, https://bevy-cheatbook.github.io/assets/assetserver.html
		let mut static_v_asset_handles = StaticVehicleAssetHandles::default();
		// Static vehicle models
		for v_type in static_data.all_vehicle_types() {
			/*let asset_path = cache::get_static_vehicle_model_path(server_addr.0, &v.name).strip_prefix("assets/").unwrap_or("<Error: vehicle model path does not start with correct dir ('assets/')>").to_owned() + "#Scene0";
			dbg!(&asset_path);
			let handle = asset_server.load(asset_path);*/
			//let handle = VehicleStaticModel::load_to_bevy(&v_type, renet_server_addr.0.ip(), &*asset_server).unwrap();
			let handle = asset_server.load(&format!("http://{:?}/vehicle_static_models/{}.glb#Scene0", asset_client.server_addr, &v_type));// TODO: use cache
			//let handle = asset_server.load("test-bike.glb#Scene0");
			//let handle = asset_server.add();// TODO
			static_v_asset_handles.0.insert(v_type.clone(), handle);
		}
		commands.insert_resource(static_v_asset_handles);
		// GLTF render test, https://bevy-cheatbook.github.io/3d/gltf.html
		/*let scene0 = asset_server.load("test_asset.glb#Scene0");
		commands.spawn(SceneBundle {// 1
			scene: scene0,
			..Default::default()
		});*/
	}
}

pub struct NetworkInitInfo {
	pub renet_transport: NetcodeClientTransport,// Trying https://doc.rust-lang.org/std/mem/fn.replace.html
	pub renet_client: RenetClient,
	pub auth: ClientAuth,
	pub renet_server_addr: net::SocketAddr
}

// Components
#[derive(Component)]
pub struct CameraComponent;

#[derive(Component)]
pub struct UsernameComponent(pub String);

/*#[derive(Component)]
pub struct VehicleNameComponent(pub String);*/

// Systems
fn vehicle_input_key_event_system(// Only for manual vehicle control should be behind the `debug_vehicle_input` feature
	keys: Res<ButtonInput<KeyCode>>,
	mut renet_client: ResMut<RenetClient>,
	auth: Res<ClientAuth>,
) {
	// Process user input (ONLY TEMPORARY)
	// Key state polling
	let input = {
		// Drive
		let power_forward: Float = if keys.pressed(KeyCode::KeyW) {
			2000.0
		}
		else {
			0.0
		};
		// Braking
		let brake = if keys.pressed(KeyCode::KeyS) {
			1.0
		}
		else {
			0.0
		};
		// Rotation/Steering
		let mut steering = 0.0;
		if keys.pressed(KeyCode::KeyA) {
			steering += 1.0;
		}
		if keys.pressed(KeyCode::KeyD) {
			steering -= 1.0;
		}
		// Create input
		InputData {
			steering,
			speed: power_forward.powf(1.0/3.0),// TODO: fix
			power: power_forward,
			brake
		}
	};
	let client_update = ClientUpdate {
		auth: auth.into_inner().clone(),
		input
	};
	renet_client.send_message(DefaultChannel::Unreliable, bincode::serialize(&RenetRequest::ClientUpdate(client_update)).unwrap());
}

fn misc_key_event_system(
	mut key_events: EventReader<KeyboardInput>,
	mut renet_client: ResMut<RenetClient>,
	auth_res: Res<ClientAuth>,
) {
	let _auth = auth_res.into_inner().clone();
	// Key events
	for ev in key_events.read() {
		match ev.state {
			ButtonState::Pressed => {
				match ev.key_code {
					KeyCode::KeyP => renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&RenetRequest::TogglePlaying).unwrap()),
					_ => {}
				}
			}
			ButtonState::Released => {
				// Unused
			}
		}
	}
}

fn vehicle_render_system(
	mut commands: Commands,
    v_static_asset_handles: Res<StaticVehicleAssetHandles>,
	mut v_scenes: Query<(&SceneRoot, &UsernameComponent, &mut Transform)>,
	world: Res<WorldSend>
) {
	// Based on https://bevy-cheatbook.github.io/3d/gltf.html
	// Update already created vehicle scene entities
	let all_vehicles: HashSet<String> = world.vehicles.keys().cloned().collect();
	let mut rendered_vehicles: HashSet<String> = HashSet::<String>::new();
	for (_, username, mut transform) in v_scenes.iter_mut() {
		match world.vehicles.get(&username.0) {
			Some(v) => {
				*transform = nalgebra_iso_to_bevy_transform(v.body_state.position);
				rendered_vehicles.insert(username.0.clone());
			},
			None => panic!("Vehicle scene entity exists with a username (\"{}\") not included in `Res<VehicleBodyStates>`", &username.0)
		}
	}
	// Create new ones
	for username in all_vehicles.difference(&rendered_vehicles) {
		let v_type = &world.vehicles.get(username).unwrap().type_;
		println!("Creating scene for \"{}\" with vehicle type \"{}\"", username, v_type);
		commands.spawn((
			SceneRoot (
				match v_static_asset_handles.0.get(v_type) {
					Some(handle) => handle.clone(),
					None => panic!("Static vehicle scene asset handle for \"{}\" does not exist in `Res<StaticVehicleAssetHandles>`", v_type)
				}
			),
			UsernameComponent(username.clone())
		));
	}
}

fn update_system(
	mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
	mut static_data: ResMut<StaticData>,
	asset_server: Res<AssetServer>,
	mut renet_client: ResMut<RenetClient>,
	mut asset_client: ResMut<AssetLoaderManager>,
	mut requested_chunks: ResMut<RequestedChunks>,
	renet_server_addr: Res<RenetServerAddr>,
	mut world_state_res: ResMut<WorldSend>,
	settings: Res<Settings>,
	mut msg_log: ResMut<message_log::Log>
) {
	// How to move a camera: https://bevy-cheatbook.github.io/cookbook/pan-orbit-camera.html
	// Incomming messages
	let mut most_recent_world_state: Option<WorldSend> = None;
	let mut messages = Vec::<Vec<u8>>::new();
	while let Some(message) = renet_client.receive_message(DefaultChannel::ReliableOrdered) {
		messages.push(message.to_vec());
	}
	while let Some(message) = renet_client.receive_message(DefaultChannel::ReliableUnordered) {
		messages.push(message.to_vec());
	}
	while let Some(message) = renet_client.receive_message(DefaultChannel::Unreliable) {
		messages.push(message.to_vec());
	}
	for message in messages {
		let res = bincode::deserialize::<RenetResponse>(&message).unwrap();
		match res {
			RenetResponse::InitState(..) => {
				panic!("Bevy app recieved `Response::InitState(..) response which should not be possible.`");
			},
			RenetResponse::WorldState(world_send) => {
				most_recent_world_state = Some(world_send);// In case multiple come through, only use the most recent one
			},
			RenetResponse::Err(err_string) => {
				panic!("Server sent following error message: {}", err_string);
			},
			RenetResponse::Message(msg) => {
				msg_log.add(msg);
			}
		}
	}
	// Asset responses
	for asset_response_result in asset_client.update() {
		match asset_response_result {
			Ok(asset_response) => match asset_response {
				AssetResponse::VehicleRawGltfData(v_static_model) => {
					//cache::save_static_vehicle_model(server_addr.0, &v_type, data).unwrap();
					v_static_model.save(renet_server_addr.0).unwrap();
				},
				AssetResponse::Chunk(chunk) => {
					//println!("Recieved chunk {:?}", &chunk.ref_);
					requested_chunks.remove(&chunk.ref_);
					match &chunk.texture_opt {
						Some(data) => {
							if settings.cache {
								data.save(renet_server_addr.0).unwrap();
							}
						},
						None => {}//panic!("Server should send chunks with texture data")
					}
					static_data.map.generic.insert_chunk_client(chunk, &mut commands, &mut meshes, &mut materials, &asset_server, renet_server_addr.0);
				},
				AssetResponse::Wait => {},
				AssetResponse::Err(e) => error!("AssetResponse::Err({})", e)
			},
			Err(e) => error!("Error from asset client: {}", e)
		}
	}
	// This will be like the "main loop"
	if let Some(world_state) = most_recent_world_state {
		// Save world state
		*world_state_res = world_state;
	}
}

fn camera_update_system(
	mut key_events: EventReader<KeyboardInput>,
	keys: Res<ButtonInput<KeyCode>>,
	mut camera_controller: ResMut<CameraController>,
	mut camera_query: Query<(&mut CameraComponent, &mut Transform, &Projection)>,
	world_state: Res<WorldSend>,
	mut render_distance: ResMut<RenderDistance>,
	auth: Res<ClientAuth>
) {
	for (_, mut transform, _) in camera_query.iter_mut() {
		// Detect motion
		// Translation
		let mut translation = V3::zeros();
		// Z
		if keys.pressed(KeyCode::KeyT) {
			translation.z -= 1.0;
		}
		if keys.pressed(KeyCode::KeyG) {
			translation.z += 1.0;
		}
		// X
		if keys.pressed(KeyCode::KeyF) {
			translation.x -= 1.0;
		}
		if keys.pressed(KeyCode::KeyH) {
			translation.x += 1.0;
		}
		// Y
		if keys.pressed(KeyCode::Space) {
			translation.y += 1.0;
		}
		if keys.pressed(KeyCode::ShiftLeft) {
			translation.y -= 1.0;
		}
		// Rotation
		let mut rotation = V2::zeros();// yaw, pitch
		if keys.pressed(KeyCode::ArrowLeft) {
			rotation.x += 1.0;
		}
		if keys.pressed(KeyCode::ArrowRight) {
			rotation.x -= 1.0;
		}
		if keys.pressed(KeyCode::ArrowUp) {
			rotation.y += 1.0;
		}
		if keys.pressed(KeyCode::ArrowDown) {
			rotation.y -= 1.0;
		}
		// Update camera controller
		// Key events
		for ev in key_events.read() {
			match ev.state {
				ButtonState::Pressed => {
					match ev.key_code {
						KeyCode::KeyC => camera_controller.toggle(&world_state.vehicles, auth.name.clone()),
						_ => {}
					}
				}
				ButtonState::Released => {
					// Unused
				}
			}
		}
		camera_controller.update(1.0/60.0, translation, rotation);
		// Update camera
		let nalgebra_iso = camera_controller.get_pos(&world_state.vehicles);
		*transform = nalgebra_iso_to_bevy_transform(nalgebra_iso.clone());
		// Update render distance
		render_distance.set_position(v3_to_v2(&nalgebra_iso.translation.vector.clone()));
	}
}

fn render_test(
	mut commands: Commands
) {
	// Copied from https://bevyengine.org/examples/3D%20Rendering/3d-scene/
	// Camera
	/*commands.spawn((
		Camera3dBundle {
			transform: Transform::from_xyz(50.0, 10.0, 0.0).looking_at(Vec3::new(50., 0., 50.), Vec3::Y),
			..default()
		},
		CameraComponent,
		Skybox(asset_server.load("Ryfjallet_cubemap.png"))
	));*/
	/*commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(50.0, 50.0, 50.0),
        ..default()
    });*/
	// directional 'sun' light, copied from https://bevyengine.org/examples/3D%20Rendering/lighting/
    commands.spawn((
		DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        Transform {
            translation: Vec3::new(0.0, 100.0, 0.0),
            rotation: Quat::from_rotation_x(-PI / 2.),
            ..default()
        },
        // The default cascade config is designed to handle large scenes.
        // As this example has a much smaller world, we can tighten the shadow
        // bounds for better visual quality.
        /*cascade_shadow_config: CascadeShadowConfigBuilder {
            first_cascade_far_bound: 4.0,
            maximum_distance: 10.0,
            ..default()
        }
        .into(),*/
	));
}

struct MainClientPlugin;

impl Plugin for MainClientPlugin {
	fn build(&self, app: &mut App) {
		//app.add_systems(Update, manual_input);
		app.add_systems(Update, misc_key_event_system);
		#[cfg(feature = "debug_vehicle_input")]
		app.add_systems(Update, vehicle_input_key_event_system);
		app.add_systems(Update, update_system);
		//app.add_plugin(InspectableRapierPlugin);
		app.add_plugins(WorldInspectorPlugin::new());
		// Rendering
		app.insert_resource(ClearColor(Color::srgb(0.7, 0.7, 0.7)));// https://bevy-cheatbook.github.io/window/clear-color.html
		app.add_systems(Startup, render_test);
		app.add_systems(Update, vehicle_render_system.after(update_system));// Vehicle rendering AFTER recieving latest world state
		app.add_systems(Update, camera_update_system.after(update_system));
	}
}

pub fn start(init_info: InitInfo) {
	let mut app = App::new();
	//app.add_plugins(bevy_web_asset::WebAssetPlugin);
	app.add_plugins((
		bevy_web_asset::WebAssetPlugin::default(),// TODO: Wait for this to get updated to bevy 15
		DefaultPlugins.set(WindowPlugin {
			primary_window: Some(Window {
				title: APP_NAME.to_string(),
				..Default::default()
			}),
			..Default::default()
		}),
		ChunkManagerPlugin,
		HardwareControllerPlugin,
		skybox::Sky{resolution: 1000},
		chat::ChatGuiPlugin,
		MainClientPlugin,
		main_egui::ForcesGuiPlugin,
		main_egui::NavGuiPlugin,
		RenetClientPlugin,
		NetcodeClientPlugin,
	));
	InitInfo::setup_app(init_info, &mut app);
	println!("Starting Bevy app");
	app.run();
}