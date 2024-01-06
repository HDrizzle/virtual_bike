// Map main module file

use std::{collections::HashMap, mem, thread, sync::{mpsc, Arc, Mutex}, time::Instant};
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
#[cfg(feature = "client")]
use bevy::prelude::*;
#[cfg(feature = "client")]
#[cfg(feature = "debug_render_physics")]
use bevy_rapier3d::plugin::RapierContext;

use crate::prelude::*;
#[cfg(feature = "debug_render_physics")]
use crate::world::PhysicsStateSend;

pub mod chunk;
pub mod path;
#[cfg(feature = "server")]
pub mod map_generation;

// Rapier 3D physics
#[cfg(any(feature = "server", feature = "debug_render_physics"))]
use rapier3d::prelude::*;

// Structs
// Dummy struct to take the same amount of memory as `MapGenerator`, so that when the client decodes it, it will still work
/*pub struct ClientDummy {
	_padding: [u8; mem::size_of::<MapGenerator>()]
}*/

// Using my new fancy macro (WOW) to create MapSend
/*#[cfg_attr(feature = "frontend", derive(Resource))]
struct_subset!(Map, MapSend, name, loaded_chunks, path_set, chunk_size, chunkgrid_size, landmarks, background_color, auto_gen_chunks, body_handle_opt);// Simple for now*/

#[derive(Serialize, Deserialize, Clone)]
#[cfg_attr(feature = "frontend", derive(Resource))]
pub struct GenericMap {// Serialize, Deserialize, Clone, used by client AND server
	pub name: String,
	//#[cfg(not(feature = "server"))] pub gen: ClientDummy,
	#[serde(skip)]
	pub loaded_chunks: Vec<Chunk>,
	pub path_set: PathSet,
	pub chunk_size: UInt,// length of the side of each chunk, world units (meters)
	pub chunk_grid_size: UInt,// Number of points along the side of each chunk
	pub landmarks: HashMap<String, V2>,
	pub background_color: [u8; 3],
	pub auto_gen_chunks: bool,
	#[serde(skip)]
	#[cfg(any(feature = "server", feature = "debug_render_physics"))]
	pub body_handle_opt: Option<RigidBodyHandle>
}

impl GenericMap {
	#[cfg(feature = "server")]
	pub fn new(name: &str, chunk_size: UInt, chunk_grid_size: UInt, background_color: [u8; 3]) -> Self {
		Self {
			name: name.to_owned(),
			loaded_chunks: Vec::<Chunk>::new(),
			path_set: PathSet::default(),
			chunk_size,
			chunk_grid_size,
			landmarks: HashMap::<String, V2>::new(),
			background_color,
			auto_gen_chunks: true,
			#[cfg(any(feature = "server", feature = "debug_render_physics"))]
			body_handle_opt: None
			/*#[cfg(feature = "server")]
			active_chunk_creators: Vec::new()*/
		}
	}
	#[cfg(any(feature = "server", feature = "debug_render_physics"))]
	pub fn init_rapier(&mut self, bodies: &mut RigidBodySet) {
		// Create immovable body
		let body = RigidBodyBuilder::kinematic_position_based().position(Iso::identity()).build();
		let body_handle = bodies.insert(body);
		self.body_handle_opt = Some(body_handle);
	}
    #[cfg(feature = "client")]
	pub fn unload_chunk_client(&mut self, ref_: &ChunkRef, #[cfg(feature = "debug_render_physics")] mut context: &mut RapierContext, meshes: &mut ResMut<Assets<Mesh>>, materials: &mut ResMut<Assets<StandardMaterial>>) {
		match self.get_chunk_id(ref_) {
			Some(i) => {
				let chunk = &mut self.loaded_chunks[i];
				// Remove Bevy mesh
				meshes.remove(chunk.asset_id_opt.expect("Client-side chunk unloading expects chunks to have Some(HandleId)"));
				// Generic unload
				#[cfg(any(feature = "server", feature = "debug_render_physics"))]
				{
					#[cfg(feature = "debug_render_physics")]
					self.unload_chunk(
						i,
						Some(&mut RapierBodyCreationDeletionContext::from_bevy_rapier_context(&mut context))
					);
					#[cfg(not(feature = "debug_render_physics"))]
					self.unload_chunk(
						i,
						None
					);
				}
				#[cfg(not(any(feature = "server", feature = "debug_render_physics")))]
				self.unload_chunk(i);
			},
			None => {}
		}
		// TODO
	}
	pub fn unload_chunk(
		&mut self,
		i: usize,
		#[cfg(any(feature = "server", feature = "debug_render_physics"))] rapier_data_opt: Option<&mut RapierBodyCreationDeletionContext>
	) {
		let chunk = &mut self.loaded_chunks[i];
		// Can't simply delete chunk, 1st must remove it from the physics engine
		#[cfg(any(feature = "server", feature = "debug_render_physics"))] match rapier_data_opt {
			Some(mut rapier_data) => chunk.remove_from_rapier(&mut rapier_data),
			None => {}
		}
		// Now we can delete it
		self.loaded_chunks.remove(i);
	}
	#[cfg(feature = "server")]
	pub fn unload_chunks(&mut self, chunks_needed: &Vec<ChunkRef>, rapier_data: &mut RapierBodyCreationDeletionContext) {
		// Unloads all chunks except the ones in `chunks_needed`
        let mut chunks_to_unload = Vec::<usize>::new();
        for (i, chunk) in self.loaded_chunks.iter().enumerate() {
            if !chunks_needed.contains(&chunk.ref_) {
				chunks_to_unload.push(i);
			}
        }
        for i in chunks_to_unload.iter().rev() {// Important to reverse the index order so its from largest to smallest
			//println!("Unloading chunk {:?}", &ref_);
            self.unload_chunk(*i, Some(rapier_data));
        }
	}
	pub fn is_chunk_loaded(&self, ref_: &ChunkRef) -> bool {
		for chunk in &self.loaded_chunks {
			if &chunk.ref_ == ref_ {
				return true;
			}
		}
		false
	}
    #[cfg(feature = "client")]
	pub fn insert_chunk_client(&mut self, mut chunk: Chunk, #[cfg(feature = "debug_render_physics")] rapier_data: &mut RapierBodyCreationDeletionContext, commands: &mut Commands, meshes:  &mut ResMut<Assets<Mesh>>, materials: &mut ResMut<Assets<StandardMaterial>>, asset_server: &AssetServer) {
		// Add to bevy rendering world
		if !self.is_chunk_loaded(&chunk.ref_) {
			chunk.bevy_pbr_bundle(commands, meshes, materials, asset_server);
		}
		// Generic insert
		#[cfg(any(feature = "server", feature = "debug_render_physics"))]
		let loaded: bool = {
			#[cfg(feature = "debug_render_physics")]
			{self.insert_chunk(
				chunk,
				Some(rapier_data)
			)}
			#[cfg(not(feature = "debug_render_physics"))]
			{self.insert_chunk(
				chunk,
				None
			)}
		};
		#[cfg(not(any(feature = "server", feature = "debug_render_physics")))]
		let loaded = self.insert_chunk(chunk);
		assert!(loaded, "self.insert_chunk says the chunk was not loaded, however a previous check says the chunk didn't already exist. This should not be possible.");
	}
	pub fn insert_chunk(&mut self, mut chunk: Chunk, #[cfg(any(feature = "server", feature = "debug_render_physics"))] rapier_data_opt: Option<&mut RapierBodyCreationDeletionContext>) -> bool {
		// IMPORTANT: THIS SHOULD BE THE ONLY PLACE WHERE self.loaded_chunks.push() IS CALLED
		if self.is_chunk_loaded(&chunk.ref_) {
			return false;//panic!("Attempt to insert already loaded chunk");
		}
		#[cfg(any(feature = "server", feature = "debug_render_physics"))]
		match rapier_data_opt {
			Some(rapier_data) => {
				let body_handle = self.body_handle_opt.expect("Map body handle is None when trying to insert chunk, help: maybe init_rapier() hasn\'t been called yet");
				chunk.init_rapier(rapier_data, body_handle);
			},
			None => {}
		}
		/*#[cfg(feature = "client")]
		assert!(chunk.asset_id_opt.is_some(), "Attempt in client to map.loaded_chunks.push() chunk with no asset id");*/
		self.loaded_chunks.push(chunk);
		true
	}
	pub fn get_chunk_id(&self, chunk_ref: &ChunkRef) -> Option<usize> {
		for (i, chunk) in self.loaded_chunks.iter().enumerate() {
			if &chunk.ref_ == chunk_ref {
				return Some(i);
			}
		}
		None
	}
	#[cfg(feature = "client")]
	pub fn init_bevy(&mut self, commands: &mut Commands, meshes:  &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>, asset_server: &AssetServer) {
		#[cfg(feature = "path_rendering")]
		for (_, path) in &mut self.path_set.paths.iter_mut() {
			path.init_bevy(commands, meshes, materials, asset_server);
		}
	}
}

#[cfg(feature = "server")]
#[derive(Serialize, Deserialize)]
pub struct SaveMap {
	pub generic: GenericMap,
	pub gen: MapGenerator,
	pub chunk_creation_rate_limit: Float
}

#[cfg(feature = "server")]
pub struct ServerMap {
	pub generic: GenericMap,
	gen: MapGenerator,
	chunk_creator: ChunkCreationManager
}

impl ServerMap {
	pub fn from_save(save: SaveMap) -> Self {
		Self {
			generic: save.generic,
			gen: save.gen,
			chunk_creator: ChunkCreationManager::new(save.chunk_creation_rate_limit)
		}
	}
	pub fn new(name: &str, chunk_size: UInt, chunk_grid_size: UInt, gen: MapGenerator, background_color: [u8; 3]) -> Self {
		Self {
			generic: GenericMap::new(
				name,
				chunk_size,
				chunk_grid_size,
				background_color
			),
			gen,
			chunk_creator: ChunkCreationManager::new(4.0)// TODO: fix arbitrary value
		}
	}
	pub fn send(&self, #[cfg(feature = "debug_render_physics")] physics: &mut PhysicsStateSend) -> GenericMap {
		// Unloads all chunks from the physics state which will also be sent to the client
		let mut out = self.generic.clone();
		#[cfg(feature = "debug_render_physics")]
		out.unload_chunks(&Vec::<ChunkRef>::new(), &mut physics.build_body_creation_deletion_context());
		// Done
		out
	}
	pub fn load_or_create_chunk(&mut self, ref_: &ChunkRef) -> Option<Chunk> {
		return if ref_.exists(&self.generic.name) {// Chunk already exists on the disk, load it
			//println!("Loading chunk {:?}", &ref_);
			Some(Chunk::load(ref_, &self.generic.name).unwrap())
		}
		else {// Chunk does not exist, create it
			if self.generic.auto_gen_chunks {
				self.chunk_creator.start(ChunkCreationArgs {
					ref_: ref_.clone(),
					size: self.generic.chunk_size,
					grid_size: self.generic.chunk_grid_size,
					gen: self.gen.clone(),
					background_color: self.generic.background_color,
					map_name: self.generic.name.clone(),
				});
			}
			None
		};
	}
	pub fn check_chunk_creator(&mut self, rapier_data: &mut RapierBodyCreationDeletionContext) -> Vec<Result<(), String>> {
		let results = self.chunk_creator.check_chunk_creators();
		let mut out = Vec::<Result<(), String>>::new();
		for res in results {
			match res {
				Ok(chunk) => {self.generic.insert_chunk(chunk, Some(rapier_data));},
				Err(e) => out.push(Err(e))
			}
		}
		// Done
		out
	}
	pub fn save(&self) -> SaveMap {
		SaveMap {
			generic: self.generic.clone(),
			gen: self.gen.clone(),
			chunk_creation_rate_limit: self.chunk_creator.rate_limit
		}
	}
}

#[cfg(feature = "server")]
struct ChunkCreationManager {
	active_chunk_creators: Vec<AsyncChunkCreator>,
	ref_request_count: HashMap<ChunkRef, u32>,
	rate_limit: Float,
	most_recent_creation: Arc<Mutex<Instant>>
}

impl ChunkCreationManager {
	pub fn new(rate_limit: Float) -> Self {
		Self {
			active_chunk_creators: Vec::new(),
			ref_request_count: HashMap::new(),
			rate_limit,// Arbitrary, TODO: get
			most_recent_creation: Arc::new(Mutex::new(Instant::now()))
		}
	}
	pub fn start(&mut self, chunk_creation_args: ChunkCreationArgs) {
		let most_recent_creation_clone = self.most_recent_creation.clone();
		self.active_chunk_creators.push(AsyncChunkCreator::start(chunk_creation_args, most_recent_creation_clone));// TODO: rate limiting, priority, and such things
	}
	pub fn check_chunk_creators(&mut self) -> Vec<Result<Chunk, String>> {
		let mut out = Vec::<Result<Chunk, String>>::new();
		for chunk_creator in &mut self.active_chunk_creators {
			let opt = chunk_creator.check();
			match opt {
				Some(result) => out.push(result),
				None => {}
			}
		}
		// Done
		out
	}
}

#[cfg(feature = "server")]
struct AsyncChunkCreator {
	start_time: Instant,
	thread_handle_opt: Option<thread::JoinHandle<()>>,
	chunk_ref: ChunkRef,
	result: Arc<Mutex<Option<Result<Chunk, String>>>>
}

impl AsyncChunkCreator {
	pub fn start(chunk_creation_args: ChunkCreationArgs, most_recent_creation: Arc<Mutex<Instant>>) -> Self {
		let result: Arc<Mutex<Option<Result<Chunk, String>>>> = Arc::new(Mutex::new(None));
		let result_clone = result.clone();
		let chunk_ref = chunk_creation_args.ref_.clone();
		let thread_handle = thread::spawn(
			move || Self::main_loop(result_clone, most_recent_creation, chunk_creation_args)
		);
		Self {
			start_time: Instant::now(),
			thread_handle_opt: Some(thread_handle),
			chunk_ref,
			result
		}
	}
	fn main_loop(result_mutex: Arc<Mutex<Option<Result<Chunk, String>>>>, most_recent_creation: Arc<Mutex<Instant>>, chunk_creation_args: ChunkCreationArgs) {
		// TODO: delay
		let result = Chunk::new(chunk_creation_args);
		*result_mutex.lock().unwrap() = Some(result);
	}
	pub fn check(&mut self) -> Option<Result<Chunk, String>> {
		if self.is_done() {// If there is a thread handle, the thread hasn't been joined yet
			let mut result_ref = self.result.lock().unwrap();
			let chunk_res_opt: Option<Result<Chunk, String>> = mem::replace(&mut *result_ref, None);
			match &chunk_res_opt {// If thread set the arc mutex to Som(_), then it is done, join it
				Some(_) => {
					let thread_handle_opt_owned = mem::replace(&mut self.thread_handle_opt, None);
					match thread_handle_opt_owned {
						Some(thread_handle_owned) => {thread_handle_owned.join().unwrap()},
						None => panic!("Impossible situation")
					}
				},
				None => {}
			}
			chunk_res_opt
		}
		else {
			None
		}
	}
	pub fn is_done(&self) -> bool {
		match &self.thread_handle_opt {
			Some(_) => false,
			None => true
		}
	}
}