// Plugin for loading/unloading chunks

use std::{time::{SystemTime, Duration}, collections::HashMap};
use bevy::prelude::*;
use bevy_renet::renet::*;
#[cfg(feature = "debug_render_physics")]
use bevy_rapier3d::plugin::RapierContext;

use crate::{prelude::*, renet_server::Request};

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
	pub pos: P2
}

impl RenderDistance {
	pub fn load_unload_chunks(
		&self,
		map: &mut Map,
		#[cfg(feature = "debug_render_physics")]
		context: &mut RapierContext,
		meshes: &mut ResMut<Assets<Mesh>>,
		materials: &mut ResMut<Assets<StandardMaterial>>
	) -> Vec<ChunkRef> {
		// Returns list of ChunkRefs that need to be loaded
		// 1: List all chunks that MUST be loaded
		// This basically the same as crate::map::chunk::Chunk::distance_to_point
		/*let chunk_grid_radius: i64 = ((self.load as UInt) / map.chunk_size) as i64 + 1;
		for dx in chunk_grid_radius..chunk_grid_radius {
			for dy in -chunk_grid_radius..chunk_grid_radius {
				let dist: Float = (((dx * (map.chunk_size as i64)).pow(2) + (dy * (map.chunk_size as i64)).pow(2)) as Float).sqrt();
				if dist <= self.load {
					let ref_ = ChunkRef{position: [dx.try_into().unwrap(), dy.try_into().unwrap()]};// TODO: add camera position
					if !map.is_chunk_loaded(&ref_) {
						out.push(ref_);
					}
				}
			}
		}*/
		// Test, only requests to load occupied and adjacent chunks
		let chunks_to_load: Vec<ChunkRef> = {
			let mut out = Vec::<ChunkRef>::new();
			let center_chunk = ChunkRef::from_world_point(p2_to_p3(self.pos.clone()), map.chunk_size);
			for ref_ in center_chunk.adjacent_chunks(map.chunk_size, false) {
				if !map.is_chunk_loaded(&ref_) {
					out.push(ref_);
				}
			}
			if !map.is_chunk_loaded(&center_chunk) {// include center chunk
				out.push(center_chunk);
			}
			out
		};
		// 1: Unload, TODO: test
		let mut chunks_to_unload = Vec::<ChunkRef>::new();
		for chunk in map.loaded_chunks.iter() {
			if (!chunks_to_load.contains(&chunk.ref_)) && self.chunk_load_status(&chunk) == ChunkLoadStatus::Unload {
				chunks_to_unload.push(chunk.ref_.clone());
			}
		}
		for ref_ in chunks_to_unload {
			map.unload_chunk_client(&ref_, #[cfg(feature = "debug_render_physics")]context, meshes, materials);
		}
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
	pub fn set_position(&mut self, pos: P2) {
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
	pub fn add(&mut self, req_ref: ChunkRef, renet_client: &mut RenetClient) {
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
		renet_client.send_message(DefaultChannel::ReliableOrdered, bincode::serialize(&Request::Chunk(req_ref)).unwrap());
	}
	pub fn remove(&mut self, ref_: &ChunkRef) {
		self.requested.remove(ref_);
	}
}

// Systems
pub fn update_chunks_system(// TODO get this to only run when main vehicle is moved
	mut meshes: ResMut<Assets<Mesh>>,
	mut materials: ResMut<Assets<StandardMaterial>>,
	render_distance: Res<RenderDistance>,
	mut static_data: ResMut<StaticData>,
	#[cfg(feature = "debug_render_physics")]
	mut rapier_context_res: ResMut<RapierContext>,
	mut renet_client: ResMut<RenetClient>,
	mut requested_chunks: ResMut<RequestedChunks>
) {
	let chunks_to_load: Vec<ChunkRef> = render_distance.load_unload_chunks(&mut static_data.map, #[cfg(feature = "debug_render_physics")] &mut rapier_context_res, &mut meshes, &mut materials);
	for chunk_ref in chunks_to_load {
		requested_chunks.add(chunk_ref, &mut renet_client);
	}
}

// Plugin
pub struct ChunkManagerPlugin;

impl Plugin for ChunkManagerPlugin {
	fn build(&self, app: &mut App) {
		app.insert_resource(RenderDistance{load: 200.0, unload: 300.0, pos: P2::new(0.0, 0.0)});// TODO: fix hardcoded values
		app.insert_resource(RequestedChunks::new(5.0));
		app.add_systems(Update, update_chunks_system);
	}
}