// For generating elevation meshes from real-world data
use std::{f64::consts::PI, default::Default};
use serde::{Serialize, Deserialize};
use serde_json;
use reqwest;

use crate::prelude::*;

// CONSTANTS
pub const EARTH_RADIUS: Float = 6.378e+6;
pub const EARTH_CIRC: Float = EARTH_RADIUS * 2.0 * (PI as Float);
pub const TC: Float = (PI / 180.0) as Float;// Trigonometry Calibration, multiply by degrees to get radians

// Request/Response structs, Conforms to the the POST API format described in https://github.com/Jorl17/open-elevation/blob/master/docs/api.md
#[derive(Serialize, Debug)]
struct LocationRequest {
	pub latitude: Float,
	pub longitude: Float
}

#[derive(Serialize, Debug)]
struct ElevationRequest {
	pub locations: Vec<LocationRequest>
}

impl ElevationRequest {
	pub fn get(&self) -> Result<ElevationResponse, String> {
		// POST
		/*
		let serialized = extras::to_string_err(serde_json::to_string(&self))?;
		dbg!(&serialized);
		let client = reqwest::blocking::Client::new();
		let res: ElevationResponse = extras::to_string_err(
			extras::to_string_err(
				client.post("https://api.open-elevation.com/api/v1/lookup")
				.body(serialized)
				.send()
			)?
			.json::<ElevationResponse>())?;*/
		// GET
		let mut req_url = "https://api.open-elevation.com/api/v1/lookup?locations=".to_string();
		for location in &self.locations {
			req_url += &format!("{},{}|", location.longitude, location.latitude);// I checked manually and trailing "|" chars are fine
		}
		//dbg!(&req_url);
		let res_raw: String = to_string_err(
			to_string_err(
				reqwest::blocking::get(req_url)
			)?
			.text())?;
		let res: ElevationResponse = to_string_err(serde_json::from_str(&res_raw))?;
		// Done
		//dbg!(&res);
		Ok(res)
	}
}

impl Default for ElevationRequest {
	fn default() -> Self {
		Self {
			locations: Vec::new()
		}
	}
}

#[derive(Deserialize, Debug)]
struct LocationResponse {
	pub latitude: Float,
	pub longitude: Float,
	pub elevation: Float
}

#[derive(Deserialize, Debug)]
struct ElevationResponse {
	pub results: Vec<LocationResponse>
}

impl ElevationResponse {
	pub fn to_elev_mesh(&self, precision: Float, grid_matrix_size: UInt) -> RegularElevationMesh {
		assert_eq!(self.results.len(), grid_matrix_size.pow(2) as usize);
		let mut grid = Vec::<Vec<Float>>::new();
		for y in 0..grid_matrix_size {
			let mut row = Vec::<Float>::new();
			for x in 0..grid_matrix_size {
				row.push(self.results[((y * grid_matrix_size) + x) as usize].elevation);
			}
			grid.push(row);
		}
		RegularElevationMesh {
			precision,
			grid
		}
	}
}

pub fn create_mesh_from_real_world(
	size: UInt,// Chunk size
	grid_size: UInt,// Chunk grid size
	chunk_ref: &ChunkRef,
	map_location: &[Float; 2]
) -> RegularElevationMesh {
	// TODO: test that edges of different chunks have same angular coords
	// Uses this api: https://github.com/Jorl17/open-elevation/blob/master/docs/api.md
	let grid_matrix_size = grid_size + 1;
	let precision = (size as Float) / (grid_size as Float);
	// Angle offset for this chunk
	//let (angles, x_scale) = chunk_local_location(chunk_ref, map_location);
	//println!("Creating chunk (ref_={:?}) from real terrain at lon={}, lat={}", chunk_ref, angles[0], angles[1]);
	// Create blank elevation request
	let mut req = ElevationRequest::default();
	for y in 0..grid_matrix_size {
		//let y_angle = angles[1] + (meters_to_degrees(y as Int) * precision);//meters_to_degrees(chunk_ref.position.1 + ((y as Float * precision) as Int));//
		for x in 0..grid_matrix_size {
			//let x_angle = angles[0] + (meters_to_degrees(x as Int) * precision) / x_scale;//meters_to_degrees(chunk_ref.position.0 + ((x as Float * precision) as Int));
			let ((x_angle, y_angle), scale) = chunk_local_location(&map_location, chunk_ref, IntP2(((x as Float * precision) as Int), ((y as Float * precision) as Int)));
			req.locations.push(LocationRequest{latitude: y_angle, longitude: x_angle});
		}
	}
	//dbg!(serde_json::to_string(&req));
	let response: ElevationResponse = req.get().unwrap();
	//dbg!(serde_json::to_string(&response));
	response.to_elev_mesh(precision, grid_matrix_size)
}

pub fn chunk_local_location(
	map_location: &[Float; 2],
	chunk_ref: &ChunkRef,
	local_offset: IntP2
) -> ((Float, Float), Float) {
	// ((X angle, Y angle), X scale)
	// Net offset from map anchor
	let net_offset = local_offset + chunk_ref.position;
	let chunk_y_angle = map_location[1] + meters_to_degrees(net_offset.1);
	// Crude way to scale X/longitude according to Y/latitude
	#[cfg(feature = "GIS_longitude_scaling")]
	let x_scale = (/*chunk_y_angle*/map_location[1] * TC).cos();
	#[cfg(not(feature = "GIS_longitude_scaling"))]
	let x_scale = 1.0;
	let chunk_x_angle = map_location[0] + (meters_to_degrees(net_offset.0) * x_scale);
	((chunk_x_angle, chunk_y_angle), x_scale)
}

pub fn meters_to_degrees(m: Int) -> Float {
	let rev = (m as Float) / EARTH_CIRC;
	rev * 360.0
}

pub fn degrees_to_meters(deg: Float) -> Int {
	let rev = deg / 360.0;
	(rev * EARTH_CIRC) as Int
}