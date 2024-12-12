//! Plugin for loading/unloading chunks

use std::{time::{SystemTime, Duration}, collections::HashMap};
use bevy::prelude::*;
#[cfg(feature = "debug_render_physics")]
use bevy_rapier3d::plugin::RapierContext;

use crate::prelude::*;

use super::super::asset_client::AssetLoaderManager;

// Structs/enums
#[derive(PartialEq)]
pub enum ChunkLoadStatus{
	Unload,
	Neutral,// Within "hysteresis" window
	Load
}

// Resources
#[derive(Resource)]
pub struct RenderDistance {
	pub load: Float,// Minimum distance for chunk to be unloaded
	pub unload: Float,// Maximum distance for chunk to be loaded
	pub pos: V2
}

impl RenderDistance {
	pub fn load_unload_chunks(
		&self,
		map: &mut GenericMap,
		#[cfg(feature = "debug_render_physics")] context: &mut RapierContext,
		meshes: &mut ResMut<Assets<Mesh>>,
		materials: &mut ResMut<Assets<StandardMaterial>>
	) -> Vec<ChunkRef> {
		// Returns list of ChunkRefs that need to be loaded
		// 1: Unload
		let mut chunks_to_unload = Vec::<ChunkRef>::new();
		for chunk in map.loaded_chunks.iter() {
			if self.chunk_load_status(&chunk) == ChunkLoadStatus::Unload {
				chunks_to_unload.push(chunk.ref_.clone());
			}
		}
		for ref_ in &chunks_to_unload {
			map.unload_chunk_client(ref_, #[cfg(feature = "debug_render_physics")] context, meshes, materials);
		}
		// 2: List all chunks that must be loaded
		let mut chunks_to_load: Vec<ChunkRef> = {
			let mut out = Vec::<ChunkRef>::new();
			let center_chunk = ChunkRef::from_world_point(self.pos.clone(), map.chunk_size);
			// Test, only requests to load occupied and adjacent chunks
			/*for ref_ in center_chunk.adjacent_chunks(map.chunk_size, false) {
				out.push(ref_);
			}
			// include center chunk
			out.push(center_chunk);*/
			// This is basically the same as crate::map::chunk::Chunk::distance_to_point
			let chunk_size_int = map.chunk_size as Int;
			let search_radius = self.load as Int + chunk_size_int;
			for x in center_chunk.position.0 - search_radius .. center_chunk.position.0 + search_radius {
				if x % chunk_size_int != 0 {continue;}
				for y in center_chunk.position.1 - search_radius .. center_chunk.position.1 + search_radius {
					if y % chunk_size_int != 0 {continue;}
					let dist: Float = (((x - center_chunk.position.0).pow(2) + (y - center_chunk.position.1).pow(2)) as Float).sqrt();// TODO: add camera position
					if dist <= self.load {
						let ref_ = ChunkRef{position: IntV2(x, y)};
						out.push(ref_);
					}
				}
			}
			out
		};
		// Make sure all `chunks_to_load` aren't in `chunks_to_unload` or already loaded
		let mut indices = Vec::<usize>::new();
		for (i, chunk_ref) in chunks_to_load.iter().enumerate() {
			if chunks_to_unload.contains(chunk_ref) || map.is_chunk_loaded(chunk_ref) {
				indices.push(i);
			}
		}
		for i in indices.iter().rev() {// Important to reverse the order of the indices
			chunks_to_load.remove(*i);
		}
		// Done
		chunks_to_load
	}
	pub fn chunk_load_status(&self, chunk: &Chunk) -> ChunkLoadStatus {
		let dist = chunk.distance_to_point(&self.pos);
		return if dist < self.load {
			ChunkLoadStatus::Load
		}
		else {
			if dist > self.unload {
				ChunkLoadStatus::Unload
			}
			else {
				ChunkLoadStatus::Neutral
			}
		};
	}
	pub fn set_position(&mut self, pos: V2) {
		self.pos = pos;
	}
}

#[derive(Resource)]
pub struct RequestedChunks {// For keeping track of what chunks have already been requested
	pub requested: HashMap<ChunkRef, SystemTime>,// chunk: time it was requested
	pub timeout: Duration
}

impl RequestedChunks {
	pub fn new(timeout: Float) -> Self {
		Self {
			requested: HashMap::new(),
			timeout: Duration::from_secs_f32(timeout)
		}
	}
	pub fn add(&mut self, req_ref: ChunkRef, asset_client: &mut AssetLoaderManager) {
		// Check if it has been already requested within the timeout
		for (ref_, t) in self.requested.iter() {
			if ref_ == &req_ref {
				if t.elapsed().unwrap() < self.timeout {// If requested less than `self.timeout` ago
					return;
				}
			}
		}
		//println!("Requested chunk {:?}", &req_ref);
		// Perform request
		self.requested.insert(req_ref.clone(), SystemTime::now());
		asset_client.request(
			AssetRequest::Chunk{
				chunk_ref: req_ref,
				with_texture: true// TODO
			}
		);
	}
	pub fn remove(&mut self, ref_: &ChunkRef) {
		self.requested.remove(ref_);
	}
}

// Systems
pub fn update_chunks_system(// TODO get this to only run when the camera is moved
	mut meshes: ResMut<Assets<Mesh>>,
	mut materials: ResMut<Assets<StandardMaterial>>,
	render_distance: Res<RenderDistance>,
	mut static_data: ResMut<StaticData>,
	#[cfg(feature = "debug_render_physics")]
	mut rapier_context_res: ResMut<RapierContext>,
	mut asset_client: ResMut<AssetLoaderManager>,
	mut requested_chunks: ResMut<RequestedChunks>
) {
	let chunks_to_load: Vec<ChunkRef> = render_distance.load_unload_chunks(&mut static_data.map.generic, #[cfg(feature = "debug_render_physics")] &mut rapier_context_res, &mut meshes, &mut materials);
	for chunk_ref in chunks_to_load {
		requested_chunks.add(chunk_ref, &mut asset_client);
	}
}

// Plugin
pub struct ChunkManagerPlugin;

impl Plugin for ChunkManagerPlugin {
	fn build(&self, app: &mut App) {
		app.insert_resource(RenderDistance{load: 300.0, unload: 1000.0, pos: V2::new(0.0, 0.0)});// TODO: fix hardcoded values
		app.insert_resource(RequestedChunks::new(5.0));
		app.add_systems(Update, update_chunks_system);
	}
}