//! For generating elevation meshes from real-world data
//! Currently uses https://www.opentopodata.org/datasets/aster
// Something interesting: https://e4ftl01.cr.usgs.gov/
use std::{f64::consts::PI, default::Default};
use serde::{Serialize, Deserialize};
use serde_json;
use reqwest;

use crate::prelude::*;

// CONSTANTS
pub const EARTH_RADIUS: Float = 6.378e+6;
pub const EARTH_CIRC: Float = EARTH_RADIUS * 2.0 * (PI as Float);
pub const TC: Float = (PI / 180.0) as Float;// Trigonometry Calibration, multiply by degrees to get radians

// Trait
pub trait ElevationGetter {
	fn new(locations: GenericElevationRequest) -> Self;
	fn get(&self) -> Result<GenericElevationResponse, String>;
}

pub type ApiClient = opentopodata_client::Getter;// Use this to set which API is used

#[derive(Serialize, Deserialize, Clone)]
pub struct WorldLocation {
	pub lat: Float,
	pub lon: Float
}

impl WorldLocation {
	pub fn csv(&self) -> String {
		format!("{},{}", self.lat, self.lon)
	}
}

pub struct GenericElevationRequest {
	pub locations: Vec<WorldLocation>
}

impl GenericElevationRequest {
	pub fn new() -> Self {
		Self {
			locations: Vec::<WorldLocation>::new()
		}
	}
}

pub struct GenericElevationResponse {
	pub elevations: Vec<Float>
}

impl GenericElevationResponse {
	pub fn to_elev_mesh(&self, precision: Float, grid_matrix_size: UInt) -> RegularElevationMesh {
		assert_eq!(self.elevations.len(), grid_matrix_size.pow(2) as usize);
		let mut grid = Vec::<Vec<Float>>::new();
		for y in (0..grid_matrix_size).rev() {
			let mut row = Vec::<Float>::new();
			for x in (0..grid_matrix_size).rev() {
				row.push(self.elevations[(((grid_matrix_size - (x+1)) * grid_matrix_size) + y) as usize]);// I swapped X and Y and now it works
			}
			grid.push(row);
		}
		RegularElevationMesh {
			precision,
			grid
		}
	}
}

pub mod open_elevation_client {
	use super::*;
	// Request/Response structs, Conforms to the the POST API format described in https://github.com/Jorl17/open-elevation/blob/master/docs/api.md
	#[derive(Serialize, Debug)]
	struct LocationRequest {
		pub latitude: Float,
		pub longitude: Float
	}

	#[derive(Serialize, Debug)]
	pub struct ElevationRequest {
		locations: Vec<LocationRequest>
	}

	impl ElevationGetter for ElevationRequest {
		fn new(req: GenericElevationRequest) -> Self {
			let mut loc_reqs = Vec::<LocationRequest>::new();
			for loc in req.locations {
				loc_reqs.push(LocationRequest{
					latitude: loc.lat,
					longitude: loc.lon
				});
			}
			// Done
			Self{locations: loc_reqs}
		}
		fn get(&self) -> Result<GenericElevationResponse, String> {
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
			Ok(res.to_generic())
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
		#[allow(unused)]
		pub latitude: Float,
		#[allow(unused)]
		pub longitude: Float,
		pub elevation: Float
	}

	#[derive(Deserialize, Debug)]
	struct ElevationResponse {
		pub results: Vec<LocationResponse>
	}

	impl ElevationResponse {
		pub fn to_generic(&self) -> GenericElevationResponse {
			let mut elevations = Vec::<Float>::new();
			for loc in &self.results {
				elevations.push(loc.elevation);
			}
			GenericElevationResponse{elevations}
		}
	}
}

pub mod opentopodata_client {
    use super::*;
	/* Example request: https://api.opentopodata.org/v1/aster30m?locations=46.583839,9.663014|47.583839,10.663014
	yields:
	{
		"results": [
			{
				"dataset": "aster30m", 
				"elevation": 1810.0, 
				"location": {
					"lat": 46.583839, 
					"lng": 9.663014
				}
			}, 
			{
				"dataset": "aster30m", 
				"elevation": 776.0, 
				"location": {
					"lat": 47.583839, 
					"lng": 10.663014
				}
			}
		], 
		"status": "OK"
	}
	*/
	#[derive(Deserialize, Debug)]
	struct Response {
		pub results: Vec<_Result>,
		#[allow(unused)]
		pub status: String
	}

	impl Response {
		pub fn to_generic(&self) -> GenericElevationResponse {
			let mut elevations = Vec::<Float>::new();
			for result in &self.results {
				elevations.push(result.elevation);
			}
			GenericElevationResponse{elevations}
		}
	}

	#[derive(Deserialize, Debug)]
	struct _Result {
		#[allow(unused)]
		pub dataset: String,
		pub elevation: Float,
		#[allow(unused)]
		pub location: Location
	}

	#[derive(Deserialize, Debug)]
	struct Location {
		#[allow(unused)]
		pub lat: Float,
		#[allow(unused)]
		pub lng: Float
	}

	pub struct Getter {
		req: GenericElevationRequest
	}

	impl ElevationGetter for Getter {
		fn new(locations: GenericElevationRequest) -> Self {
			Self {
				req: locations
			}
		}
		fn get(&self) -> Result<GenericElevationResponse, String> {
			// GET
			let mut req_url = "https://api.opentopodata.org/v1/aster30m?locations=".to_string();
			for location in &self.req.locations {
				req_url += &format!("{}|", location.csv());// I checked manually and trailing "|" chars are fine
			}
			//dbg!(&req_url);
			let res_raw: String = to_string_err(
				to_string_err(
					reqwest::blocking::get(req_url)
				)?
				.text())?;
			let res: Response = to_string_err_with_message(serde_json::from_str::<Response>(&res_raw), &format!("Attempted to deserialize \"{}\"", &res_raw))?;
			// Done
			//dbg!(&res);
			Ok(res.to_generic())
		}
	}
}

pub fn create_mesh_from_real_world<T: ElevationGetter>(
	size: UInt,// Chunk size
	grid_size: UInt,// Chunk grid size
	chunk_ref: &ChunkRef,
	map_location: &WorldLocation
) -> RegularElevationMesh {
	// TODO: test that edges of different chunks have same angular coords
	// Uses this api: https://github.com/Jorl17/open-elevation/blob/master/docs/api.md
	let grid_matrix_size = grid_size + 1;
	let precision = (size as Float) / (grid_size as Float);
	// Angle offset for this chunk
	//let (angles, x_scale) = chunk_local_location(chunk_ref, map_location);
	//println!("Creating chunk (ref_={:?}) from real terrain at lon={}, lat={}", chunk_ref, angles[0], angles[1]);
	// Create blank elevation request
	let mut req = GenericElevationRequest::new();
	for y in 0..grid_matrix_size {
		//let y_angle = angles[1] + (meters_to_degrees(y as Int) * precision);//meters_to_degrees(chunk_ref.position.1 + ((y as Float * precision) as Int));//
		for x in 0..grid_matrix_size {
			//let x_angle = angles[0] + (meters_to_degrees(x as Int) * precision) / x_scale;//meters_to_degrees(chunk_ref.position.0 + ((x as Float * precision) as Int));
			let ((x_angle, y_angle), _) = chunk_local_location(&map_location, chunk_ref, IntV2((x as Float * precision) as Int, (y as Float * precision) as Int));
			req.locations.push(WorldLocation{lat: y_angle, lon: x_angle});
		}
	}
	//dbg!(serde_json::to_string(&req));
	let getter: T = T::new(req);
	let response: GenericElevationResponse = getter.get().unwrap();
	//dbg!(serde_json::to_string(&response));
	response.to_elev_mesh(precision, grid_matrix_size)
}

pub fn chunk_local_location(
	map_location: &WorldLocation,
	chunk_ref: &ChunkRef,
	local_offset: IntV2
) -> ((Float, Float), Float) {
	// ((X angle, Y angle), X scale)
	// Net offset from map anchor
	let net_offset = local_offset + chunk_ref.position;
	let chunk_y_angle = map_location.lat + meters_to_degrees(net_offset.1);
	// Crude way to scale X/longitude according to Y/latitude
	let x_scale = (/*chunk_y_angle*/map_location.lat * TC).cos();
	let chunk_x_angle = map_location.lon + (meters_to_degrees(net_offset.0) * x_scale);
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

#[cfg(test)]
mod tests {
	use super::*;
	use approx::assert_relative_eq;
	#[test]
	fn conversion() {
		assert_relative_eq!(meters_to_degrees((EARTH_RADIUS * 2.0 * (PI as Float)) as Int), 360.0, epsilon = EPSILON);
		assert_eq!(degrees_to_meters(180.0), (EARTH_RADIUS * (PI as Float)) as Int);
	}
	#[test]
	fn composition() {
		assert_eq!(degrees_to_meters(meters_to_degrees(1000)), 1000);
		assert_relative_eq!(meters_to_degrees(degrees_to_meters(1.0)), 1.0, epsilon = EPSILON);
		assert_relative_eq!(meters_to_degrees(degrees_to_meters(180.0)), 180.0, epsilon = EPSILON);
	}
	#[test]
	fn chunk_edge_alignment() {
		let map_anchor = WorldLocation{lat: 60.0, lon: -69.0};
		assert_eq!(
			chunk_local_location(&map_anchor, &ChunkRef{position: IntV2(0, 0)}, IntV2(1000, 1000)),
			chunk_local_location(&map_anchor, &ChunkRef{position: IntV2(1000, 1000)}, IntV2(0, 0))
		)
	}
	#[test]
	fn longitude_scaling() {
		assert_eq!(
			chunk_local_location(&WorldLocation{lat: 0.0, lon: 0.0}, &ChunkRef::origin(), IntV2(1000, 1000)),
			((meters_to_degrees(1000), meters_to_degrees(1000)), 1.0)
		);
		assert_eq!(
			chunk_local_location(&WorldLocation{lat: 60.0, lon: 10.0}, &ChunkRef::origin(), IntV2(1000, 1000)),
			((10.0 + meters_to_degrees(500), 60.0 + meters_to_degrees(1000)), 0.49999997)
		);
	}
}