/* Bevy-based client app
Rapier plugin example (2D but similar enough): https://www.youtube.com/watch?v=GwlZ5EPu8l0
Rapier inspector (old version because current version failed build): https://docs.rs/crate/bevy-inspector-egui-rapier/latest
TODO: sometime when this gets more developed use states: https://bevy-cheatbook.github.io/programming/states.html
Bevy 3D rendering simple example: https://bevyengine.org/examples/3D%20Rendering/3d-scene/

Major change 2023-11-21: this module will only be used when the game is signed-in and being played
*/

use std::{collections::{HashMap, HashSet}, net::IpAddr};
use bevy::{
	prelude::*,
	input::{keyboard::KeyboardInput, ButtonState},
	render::{
		render_resource::WgpuFeatures,
		settings::{RenderCreation, WgpuSettings},
		RenderPlugin,
	},
	pbr::wireframe::WireframePlugin, gltf::Gltf
};
use bevy_renet::renet::{RenetClient, DefaultChannel, transport::NetcodeClientTransport};
#[cfg(feature = "debug_render_physics")]
use bevy_rapier3d::plugin::RapierContext;
#[cfg(feature = "debug_render_physics")]
use rapier3d::dynamics::RigidBodyHandle;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bincode;

use crate::{
	prelude::*,
	renet_server::{Request, Response}
};

use super::{hardware_controller::HardwareControllerPlugin, network::CustomRenetPlugin, cache};

// Mods
mod chunk_manager;
use chunk_manager::{ChunkManagerPlugin, RenderDistance, RequestedChunks};
mod gui;
use gui::GuiPlugin;

// Custom resources
/*#[derive(Resource)]
#[cfg(feature = "debug_render_physics")]
pub struct VehicleBodyHandles(pub HashMap<String, RigidBodyHandle>);*/

#[derive(Resource)]
#[cfg(feature = "debug_render_physics")]
struct MapBodyHandle(pub RigidBodyHandle);

#[derive(Resource)]
pub struct ServerAddr(pub IpAddr);

#[derive(Resource, Default, Debug)]
struct StaticVehicleAssetHandles(pub HashMap<String, Handle<Scene>>);// Vehicle type, scene handle

/*#[derive(Resource, Default)]
struct VehicleBodyStates(pub HashMap<String, BodyStateSerialize>);

impl VehicleBodyStates {
	pub fn update_from_world_state(&mut self, world: &WorldSend) {
		for (name, v_send) in world.vehicles.iter() {
			self.0.insert(name.to_owned(), v_send.body_state.clone());
		}
	}
}*/

pub struct InitInfo {// Provided by the sign-in window, DOES NOT CHANGE
	pub network: NetworkInitInfo,
	pub static_data: StaticData,
}

impl InitInfo {
	pub fn setup_app(init_info: InitInfo, app: &mut App) {
		app.insert_resource(init_info.static_data);
		app.insert_resource(init_info.network.renet_transport);
		app.insert_resource(init_info.network.renet_client);
		app.insert_resource(init_info.network.auth);
		app.insert_resource(ServerAddr(init_info.network.addr));
		app.add_systems(Startup, Self::setup_system);
		app.insert_resource(WorldSend::default());
	}
	pub fn setup_system(
		mut commands: Commands,
		mut renet_client: ResMut<RenetClient>,
		#[cfg(feature = "debug_render_physics")] mut rapier_context_res: ResMut<RapierContext>,
		mut static_data: ResMut<StaticData>,
		mut meshes: ResMut<Assets<Mesh>>,
		mut materials: ResMut<Assets<StandardMaterial>>,
		asset_server: Res<AssetServer>,
		server_addr: Res<ServerAddr>
	) {
		// Map init bevy
		static_data.map.init_bevy(&mut commands, meshes.as_mut(), materials.as_mut(), &*asset_server);
		// Load static vehicle gltf model files. This is just to have the asset handles for each vehicle type, not to display anything
		// https://bevy-cheatbook.github.io/3d/gltf.html, https://bevy-cheatbook.github.io/assets/assetserver.html
		let mut static_v_asset_handles = StaticVehicleAssetHandles::default();
		for (v_type, v) in static_data.static_vehicles.iter() {
			let asset_path = cache::get_static_vehicle_model_path(server_addr.0, &v.name).strip_prefix("assets/").unwrap_or("<Error: vehicle model path does not start with correct dir ('assets/')>").to_owned() + "#Scene0";
			dbg!(&asset_path);
			let handle = asset_server.load(asset_path);
			static_v_asset_handles.0.insert(v_type.clone(), handle);
		}
		//dbg!(&static_v_asset_handles);
		commands.insert_resource(static_v_asset_handles);
		// GLTF render test, https://bevy-cheatbook.github.io/3d/gltf.html
		/*let scene0 = asset_server.load("test_asset.glb#Scene0");
		commands.spawn(SceneBundle {// 1
			scene: scene0,
			..Default::default()
		});*/
		// Init rapier
		#[cfg(feature = "debug_render_physics")]
		{
			// (HashMap<String, RigidBodyHandle>, Vec<(String, Wheel)>)
			let mut rapier_context = rapier_context_res.into_inner();
			static_data.init_bevy_rapier_context(&mut rapier_context);
			// Save map body handle
			commands.insert_resource(MapBodyHandle(static_data.map.body_handle_opt.expect("Could not get map body handle")));
		}
	}
}

pub struct NetworkInitInfo {
	pub renet_transport: NetcodeClientTransport,// Trying https://doc.rust-lang.org/std/mem/fn.replace.html
	pub renet_client: RenetClient,
	pub auth: ClientAuth,
	pub addr: IpAddr
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
	keys: Res<Input<KeyCode>>,
	mut key_events: EventReader<KeyboardInput>,
	mut renet_client: ResMut<RenetClient>,
	mut auth: ResMut<ClientAuth>,
) {
	// Process user input (ONLY TEMPORARY)
	// Key state polling
	let input = {
		// Drive
		let power_forward: Float = if keys.pressed(KeyCode::W) {
			500.0
		}
		else {
			0.0
		};
		// Braking
		let brake = if keys.pressed(KeyCode::S) {
			0.2
		}
		else {
			0.0
		};
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

fn vehicle_render_system(
	mut commands: Commands,
    v_static_asset_handles: Res<StaticVehicleAssetHandles>,
	mut v_scenes: Query<(&Handle<Scene>, &UsernameComponent, &mut Transform)>,
	world: Res<WorldSend>
) {
	// Based on https://bevy-cheatbook.github.io/3d/gltf.html
	// Update already created vehicle scene entities
	let all_vehicles: HashSet<String> = world.vehicles.keys().cloned().collect();
	let mut rendered_vehicles: HashSet<String> = HashSet::<String>::new();
	for (handle, username, mut transform) in v_scenes.iter_mut() {
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
			SceneBundle {// TODO: include UsernameComponent
				scene: match v_static_asset_handles.0.get(v_type) {
					Some(handle) => handle.clone(),
					None => panic!("Static vehicle scene asset handle for \"{}\" does not exist in `Res<StaticVehicleAssetHandles>`", v_type)
				},
				..Default::default()
			},
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
	#[cfg(feature = "debug_render_physics")] mut rapier_context_res: ResMut<RapierContext>,
	mut camera_query: Query<(&mut CameraComponent, &mut Transform, &Projection)>,
	#[cfg(feature = "debug_render_physics")] map_body_handle: Res<MapBodyHandle>,
	mut requested_chunks: ResMut<RequestedChunks>,
	mut render_distance: ResMut<RenderDistance>,
	auth: Res<ClientAuth>,
	server_addr: Res<ServerAddr>,
	mut world_state_res: ResMut<WorldSend>
) {
	// How to move a camera: https://bevy-cheatbook.github.io/cookbook/pan-orbit-camera.html
	#[cfg(feature = "debug_render_physics")]
	let mut rapier_context = rapier_context_res.into_inner();
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
			Response::InitState(..) => {
				panic!("Bevy app recieved `Response::InitState(..) response which should not be possible.`");
			},
			Response::VehicleRawGltfData(v_type, data) => {
				cache::save_static_vehicle_model(server_addr.0, &v_type, data).unwrap();
			}
			Response::WorldState(world_send) => {
				most_recent_world_state = Some(world_send);// In case multiple come through, only use the most recent one
			},
			Response::Chunk(chunk) => {
				//println!("Recieved chunk");//, but doing nothing else because debug reasons");
				requested_chunks.remove(&chunk.ref_);
				static_data.map.insert_chunk_client(chunk, #[cfg(feature = "debug_render_physics")] &mut RapierBodyCreationDeletionContext::from_bevy_rapier_context(&mut rapier_context), &mut commands, &mut meshes, &mut materials, &asset_server);
			},
			Response::Err(err_string) => {
				panic!("Server sent following error message: {}", err_string);
			}
		}
	}
	// This will be like the "main loop"
	if let Some(world_state) = most_recent_world_state {
		// Update all bevy things, first get vect of wheels
		#[cfg(feature = "debug_render_physics")]
		world_state.update_bevy_rapier_context(&mut rapier_context, Some(map_body_handle.0));
		// Update camera position
		let main_vehicle = world_state.vehicles.get(&auth.name).expect(&format!("Unable to get vehicle for signed-in username: \"{}\"", &auth.name));
		render_distance.set_position(p3_to_p2(matrix_to_opoint(main_vehicle.body_state.position.translation.vector.clone())));
		for (_, mut transform, projection) in camera_query.iter_mut() {
			// Comment this out and add manual_camera_control() for debugging
			#[cfg(not(feature = "debug_camera_control"))]
			main_vehicle.update_bevy_camera_transform(&mut transform);
		}
		#[cfg(feature = "debug_render_physics")]
		rapier_context.propagate_modified_body_positions_to_colliders();// Just what I needed https://docs.rs/bevy_rapier3d/latest/bevy_rapier3d/plugin/struct.RapierContext.html#method.propagate_modified_body_positions_to_colliders
		// Save world state
		*world_state_res = world_state;
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
		transform.translation += vel_trans.translation * 0.5;// Very sloppy, TODO: fix
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
		app.add_systems(Update, update_system);
		// bevy_rapier3d
		#[cfg(feature = "debug_render_physics")]
		{
			app.insert_resource(bevy_rapier3d::plugin::RapierContext::default());
			app.add_plugins(bevy_rapier3d::render::RapierDebugRenderPlugin::default());
		}
		//app.add_plugin(InspectableRapierPlugin);
		app.add_plugins(WorldInspectorPlugin::new());
		// Rendering
		app.insert_resource(ClearColor(Color::rgb(0.7, 0.7, 0.7)));// https://bevy-cheatbook.github.io/window/clear-color.html
		app.add_systems(Startup, render_test);
		app.add_systems(Update, vehicle_render_system.after(update_system));// Vehicle rendering AFTER recieving latest world state
	}
}

pub fn start(init_info: InitInfo) {
	let mut app = App::new();
	app.add_plugins((
		DefaultPlugins/*.set(RenderPlugin {// From https://github.com/bevyengine/bevy/blob/main/examples/3d/wireframe.rs
			render_creation: RenderCreation::Automatic(WgpuSettings {
				// WARN this is a native only feature. It will not work with webgl or webgpu
				features: WgpuFeatures::POLYGON_MODE_LINE,
				..default()
			}),
		}),
		// You need to add this plugin to enable wireframe rendering
		WireframePlugin*/,
		CustomRenetPlugin,
		ChunkManagerPlugin,
		GuiPlugin,
		HardwareControllerPlugin,
		MainClientPlugin
	));
	InitInfo::setup_app(init_info, &mut app);
	println!("Starting Bevy app");
	app.run();
}