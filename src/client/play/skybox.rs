// For having different weather in bevy
// Example: https://github.com/bevyengine/bevy/blob/main/examples/3d/skybox.rs
// wgpu error: https://www.reddit.com/r/bevy/comments/1588577/how_to_skybox/

use std::fs;
use nalgebra::SimdValue;
use serde::{Serialize, Deserialize};
use bevy::{
	prelude::*,
	asset::LoadState,
    core_pipeline::Skybox,
	render::{render_resource::{TextureViewDescriptor, TextureViewDimension}, texture::{ImageType, CompressedImageFormats, ImageSampler}}
};
use image::{Rgb, RgbImage};

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

#[derive(Resource, Clone)]
pub struct Sky {
	pub resolution: u32
}

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
			bevy::log::info!("Skybox texture loaded");
			let image = images.get_mut(&cubemap.image_handle).unwrap();
			image.reinterpret_stacked_2d_as_array(image.height() / image.width());
            image.texture_view_descriptor = Some(TextureViewDescriptor {
                dimension: Some(TextureViewDimension::Cube),
                ..default()
            });
			bevy::log::info!("Spawning camera with skybox");
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
		let mut compiled_image = RgbImage::new(self.resolution, self.resolution * 6);
		for x in 0..self.resolution {
			for y in 0..self.resolution * 6 {
				let direction = self.stacked_skybox_pos_to_simple_rot(x, y);
				let color = Self::color_scale((direction.pitch + (PI/2.0) / PI));
				compiled_image.put_pixel(x, y, color);
			}
		}
		//image_to_bevy_image(image::open("assets/Ryfjallet_cubemap.png").unwrap().as_rgb8().expect("Expected image"))
		image_to_bevy_image(&compiled_image)
		//Image::from_buffer(&fs::read("assets/Ryfjallet_cubemap.png").unwrap()[..], ImageType::Extension("png"), CompressedImageFormats::NONE, true, ImageSampler::Default).unwrap()
	}
	/// Converts coords on image used for bevy skybox to a SimpleRotation representing where it is relative to the camera
	/// Stacked image order (from top) is:
	/// 0: +X
	/// 1: -X
	/// 2: Top (W/ image origin in (+X, +Z) orientation)
	/// 3: Bottom (W/ image origin in (-X, -Z) orientation)
	/// 4: -Z
	/// 5: +Z
	/// Bevy coord system: https://bevy-cheatbook.github.io/fundamentals/coords.html
	fn stacked_skybox_pos_to_simple_rot(&self, px_x: u32, px_y_absolute: u32) -> SimpleRotation {
		assert!(px_x < self.resolution);
		assert!(px_y_absolute < self.resolution * 6);
		// Determine which cube face the coord is in
		let image_i = px_y_absolute / self.resolution;
		let px_y = px_y_absolute % self.resolution;
		// Get image unit pos
		let resolution_float = self.resolution as Float;
		let image_pos = (V2::new(// -1 to 1
			px_x as Float / resolution_float,
			(self.resolution - px_y) as Float / resolution_float
		) * 2.0) - V2::new(1.0, 1.0);
		// Get global pos, each cube face is represented as 1 unit away from the origin
		let global_pos: V3 = match image_i {// TODO: verify
			0 => V3::new(1.0, image_pos.y, image_pos.x),
			1 => V3::new(-1.0, image_pos.y, -image_pos.x),
			2 => V3::new(image_pos.x, 1.0, image_pos.y),
			3 => V3::new(image_pos.x, -1.0, image_pos.y),
			4 => V3::new(image_pos.x, image_pos.y, -1.0),
			5 => V3::new(-image_pos.x, image_pos.y, 1.0),
			_ => panic!("Image index should be < 6, something is wrong with the code in this function")
		};
		// Done
		SimpleRotation::from_v3(global_pos)
	}
	fn color_scale(t: Float) -> Rgb<u8> {
		Rgb([
			((1.0-t) * 255.0) as u8,
			((1.0-t).min(t) * 510.0) as u8,
			(t * 255.0) as u8
		])
	}
}

impl Plugin for Sky {
	fn build(&self, app: &mut App) {
		app.insert_resource(self.clone());
		app.add_systems(Startup, Self::skybox_begin_load);
		app.add_systems(Update, Self::check_if_loaded);
	}
}