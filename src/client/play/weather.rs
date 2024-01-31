// For having different weather in bevy

use serde::{Serialize, Deserialize};
use bevy::prelude::*;
use image::Rgb;

use crate::prelude::*;

#[derive(Serialize, Deserialize, Resource)]
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
	orbit_progress_angle: Float// 0 - 1
}

#[derive(Serialize, Deserialize, Resource)]
pub struct WeatherStatic {// "Climate" and celestial objects
	sky_objects: Vec<SolarSystemObjectStatic>,
	rain_duty_cycle: Float,
	ambient_fog: Float,
	cloud_cover: Float
}

#[derive(Serialize, Deserialize, Resource)]
pub struct Weather {
	sky_objects: Vec<SolarSystemObject>,
	curr_random_state: Float,
}