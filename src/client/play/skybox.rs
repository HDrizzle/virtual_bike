// For having different weather in bevy
// Example: https://github.com/bevyengine/bevy/blob/main/examples/3d/skybox.rs
// wgpu error: https://www.reddit.com/r/bevy/comments/1588577/how_to_skybox/

use std::fs;
use serde::{Serialize, Deserialize};
use bevy::{
	prelude::*,
	asset::LoadState,
    core_pipeline::Skybox,
	render::{render_resource::{TextureViewDescriptor, TextureViewDimension}, texture::{ImageType, CompressedImageFormats, ImageSampler}}
};
use image::Rgb;

use crate::prelude::*;
use super::CameraComponent;

/*#[derive(Serialize, Deserialize, Resource)]
pub struct SolarSystemObjectStatic {
	radius: Float,
	dist_from_world: Float,
	orbit_period: Float,
	orbit_axis: V3,
	color: [u8; 3],
	albedo: Float
}

#[derive(Serialize, Deserialize, Resource)]
pub struct SolarSystemObject {
	orbit_progress: Float// 0 - 1
}

#[derive(Serialize, Deserialize, Resource)]
pub struct WeatherStatic {// "Climate" and solar system objects
	sky_objects: Vec<SolarSystemObjectStatic>,
	rain_duty_cycle: Float,
	ambient_fog: Float,
	cloud_cover: Float
}

#[derive(Serialize, Deserialize, Resource)]
pub struct Weather {
	sky_objects: Vec<SolarSystemObject>,
	curr_random_state: Float,
}*/

#[derive(Resource)]
struct Cubemap {
    is_loaded: bool,
    image_handle: Handle<Image>,
}

impl Cubemap {
	pub fn new(image_handle: Handle<Image>) -> Self {
		Self {
			is_loaded: false,
			image_handle
		}
	}
}

#[derive(Resource)]
pub struct Sky;

impl Sky {
	fn skybox_begin_load(
		mut commands: Commands,
		asset_server: Res<AssetServer>,
		sky: Res<Self>
	) {
		let cubemap = Cubemap::new(asset_server.add(sky.create_skybox_texture()));//).load("Ryfjallet_cubemap.png"));
		commands.insert_resource(cubemap);
	}
	fn check_if_loaded(
		mut commands: Commands,
		asset_server: Res<AssetServer>,
    	mut images: ResMut<Assets<Image>>,
		mut cubemap: ResMut<Cubemap>,
		mut skyboxes: Query<&mut Skybox>
	) {
		if !cubemap.is_loaded && asset_server.load_state(&cubemap.image_handle) == LoadState::Loaded {
			info!("Skybox texture loaded");
			let image = images.get_mut(&cubemap.image_handle).unwrap();
			image.reinterpret_stacked_2d_as_array(image.height() / image.width());
            image.texture_view_descriptor = Some(TextureViewDescriptor {
                dimension: Some(TextureViewDimension::Cube),
                ..default()
            });
			info!("Spawning camera with skybox");
			commands.spawn((
				Camera3dBundle {
					transform: Transform::from_xyz(50.0, 10.0, 0.0).looking_at(Vec3::new(50., 0., 50.), Vec3::Y),
					..default()
				},
				CameraComponent,
				Skybox(cubemap.image_handle.clone())
			));
			cubemap.is_loaded = true;
		}
	}
	fn create_skybox_texture(&self) -> Image {
		Image::from_buffer(&fs::read("assets/Ryfjallet_cubemap.png").unwrap()[..], ImageType::Extension("png"), CompressedImageFormats::NONE, true, ImageSampler::Default).unwrap()
	}
}

impl Plugin for Sky {
	fn build(&self, app: &mut App) {
		app.insert_resource(Self);
		app.add_systems(Startup, Self::skybox_begin_load);
		app.add_systems(Update, Self::check_if_loaded);
	}
}