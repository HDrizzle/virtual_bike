//! Map main module file

use std::{collections::HashMap, mem, thread, sync::{Arc, Mutex}, time::{Instant, Duration}, ops::Index, net::IpAddr};
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
use serde_json;
#[cfg(feature = "client")]
use bevy::prelude::*;
#[cfg(feature = "client")]
#[cfg(feature = "debug_render_physics")]
use bevy_rapier3d::plugin::RapierContext;

use crate::prelude::*;
#[cfg(feature = "debug_render_physics")]
use crate::world::PhysicsStateSend;
#[cfg(all(feature = "server", feature = "client"))]
use validity::{ValidityTest, AutoFix, ValidityTestResult, ResourceDeserializationChecker};
#[cfg(feature = "server")]
use std::fs;

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

/// Generic map struct which is Serialize/Deserialize-able and also is used during runtime, contains most of the map functionality
/// This struct can't have things such as handles for async tasks to create chunks since those can't be serialized / deserialized
#[derive(Serialize, Deserialize, Clone)]
pub struct GenericMap {// Serialize, Deserialize, Clone, used by client AND server
	pub name: String,
	//#[cfg(not(feature = "server"))] pub gen: ClientDummy,
	#[serde(skip)]
	pub loaded_chunks: Vec<Chunk>,
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
	pub fn insert_chunk_client(&mut self, mut chunk: Chunk, #[cfg(feature = "debug_render_physics")] rapier_data: &mut RapierBodyCreationDeletionContext, commands: &mut Commands, meshes:  &mut ResMut<Assets<Mesh>>, materials: &mut ResMut<Assets<StandardMaterial>>, asset_server: &AssetServer, server_addr: IpAddr) {
		// Add to bevy rendering world
		if !self.is_chunk_loaded(&chunk.ref_) {
			chunk.bevy_pbr_bundle(commands, meshes, materials, asset_server, server_addr);
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
	pub fn init_bevy(&mut self, path_set: &PathSet, commands: &mut Commands, meshes:  &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>, asset_server: &AssetServer) {
		#[cfg(feature = "path_rendering")]
		for (_, path) in &path_set.paths {
			path.init_bevy(commands, meshes, materials, asset_server);
		}
	}
	#[cfg(all(feature = "server", feature = "client"))]
	pub fn validate_chunk(&self, path: &str) -> ValidityTestResult<SaveMapAutoFix> {
		let deserialize_result = &ResourceDeserializationChecker::<Chunk>::new(path.to_owned()).check()[0];
		match deserialize_result {
			ValidityTestResult::Ok => {
				let chunk: Chunk = serde_json::from_str(&fs::read_to_string(path).unwrap()).expect("Deserializa checker said file could be deserialized, but it can't");
				// Check chunk size and grid size
				if chunk.size != self.chunk_size || chunk.grid_size != self.chunk_grid_size {
					ValidityTestResult::problem(
						validity::ProblemType::Error,
						format!("Chunk at \"{}\" has a size/grid size ({}, {}) that are inconsistent with the map's chunk size/chunk grid size ({}, {})", path, chunk.size, chunk.grid_size, self.chunk_size, self.chunk_grid_size),
						None
					)
				}
				else {
					ValidityTestResult::Ok
				}
			},
			ValidityTestResult::Problem{type_, message, ..} => ValidityTestResult::problem(
				type_.clone(),
				message.clone(),
				None
			)
		}
	}
}

#[derive(Serialize, Deserialize, Clone)]
pub struct SendMap {
	pub generic: GenericMap,
	pub path_set: PathSet
}

impl SendMap {
	#[cfg(feature = "client")]
	pub fn init_bevy(&mut self, commands: &mut Commands, meshes:  &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>, asset_server: &AssetServer) {
		self.generic.init_bevy(&self.path_set, commands, meshes, materials, asset_server);
	}
}

/// Serialize/Deserialize-able struct which is loaded from and saved to the disk
#[cfg(feature = "server")]
#[derive(Serialize, Deserialize)]
pub struct SaveMap {
	pub generic: GenericMap,
	pub gen: MapGenerator,
	pub path_set: SavePathSet,
	pub chunk_creation_rate_limit: Float,
	pub active_chunk_creators_limit: usize
}

#[cfg(all(feature = "server", feature = "client"))]
impl validity::ValidityTest for SaveMap {
	type AutoFixT = SaveMapAutoFix;
	fn condition_description(&self) -> String {
		"chunk and (optional) generic chunk files exist, chunks all have the same size/grid size, and their edges all line up".to_owned()
	}
	fn check(&self) -> Vec<ValidityTestResult<Self::AutoFixT>> {
		// Stage 1: Existance of chunks folder and generic chunk is valid (if it exists)
		let mut out = Vec::<ValidityTestResult<Self::AutoFixT>>::new();
		let mut abort = false;
		// Existance of chunks folder
		let chunks_dir = format!("{}{}/chunks", resource_interface::MAPS_DIR, &self.generic.name);
		match fs::read_dir(&chunks_dir) {
			Ok(dir_reader) => {// All folders in chunks dir have valid names and are on the chunk size grid
				for entry_res in dir_reader{
					let entry = entry_res.unwrap();
					let entry_path_binding = entry.path();
					let entry_path_str: &str = entry_path_binding.to_str().expect("couldn't convert from os string");
					if entry.metadata().unwrap().is_dir() {
						let entry_file_binding = entry.file_name();
						let entry_dir_name: &str = entry_file_binding.to_str().expect("couldn't convert from os string");
						match ChunkRef::from_resource_dir_name(entry_dir_name) {
							Ok(chunk_ref) => {
								// Check that the chunk ref is on the chunk size grid
								let mut on_grid = true;
								for ci in 0..2_usize {
									if chunk_ref.position[ci] % (self.generic.chunk_size as Int) != 0 {
										on_grid = false;
									}
								}
								if on_grid {
									// Check that the chunk's reference is the same as the one derived from the name of it's directory
									// TODO
								}
								else {
									out.push(ValidityTestResult::problem(
										validity::ProblemType::Error,
										format!("The chunk reference ({:?}) corresponding to \"{}\" does not lay on the chunk size grid ({})", &chunk_ref, entry_path_str, self.generic.chunk_size),
										None
									));
								}
							},
							Err(e) => out.push(ValidityTestResult::problem(
								validity::ProblemType::Error,
								format!("Could not decode chunk position from entry \"{}\": {}", entry_dir_name, e),
								None
							))
						}
					}
					else {
						out.push(ValidityTestResult::problem(
							validity::ProblemType::Warning,
							format!("There is a non-directory entry \"{}\" in a chunks folder", entry_path_str),
							None
						))
					}
				}
			},
			Err(e) => {
				abort = true;
				out.push(ValidityTestResult::Problem {
					type_: validity::ProblemType::Error,
					message: format!("Chunks directory (\"{}\") doesn't exist", chunks_dir),
					auto_fix_opt: Some(
						SaveMapAutoFix::new(
							&self.generic.name,
							self.generic.chunk_size,
							SaveMapAutoFixEnum::CreateChunksFolder(validity::DirectoryExistanceAutoFix::new(chunks_dir))
						)
					)
				});
			}
		}
		// Stage 2
		
		
		// Done
		out
	}
}

#[cfg(all(feature = "server", feature = "client"))]
#[derive(Clone)]
pub struct SaveMapAutoFix {
	map_name: String,
	chunk_size: UInt,
	fix: SaveMapAutoFixEnum
}

#[cfg(all(feature = "server", feature = "client"))]
impl SaveMapAutoFix {
	pub fn new(map_name: &str, chunk_size: UInt, fix: SaveMapAutoFixEnum) -> Self {
		Self {
			map_name: map_name.to_owned(),
			chunk_size,
			fix
		}
	}
}

#[cfg(all(feature = "server", feature = "client"))]
impl validity::AutoFix for SaveMapAutoFix {
	fn description(&self) -> String {
		self.fix.description(&self.map_name)
	}
	fn fix(&self) -> Result<(), String> {
		self.fix.fix(&self.map_name, self.chunk_size)
	}
}

#[derive(Clone)]
#[cfg(all(feature = "server", feature = "client"))]
pub enum SaveMapAutoFixEnum {
	DeleteEmptyGenericChunkFolder,
	DeleteEmptyChunkFolder(ChunkRef),
	CreateChunksFolder(validity::DirectoryExistanceAutoFix),
	AlignChunkEdges(ChunkRef, ChunkRef)
}

#[cfg(all(feature = "server", feature = "client"))]
impl SaveMapAutoFixEnum {
	fn description(&self, map_name: &str) -> String {
		match self {
			Self::DeleteEmptyGenericChunkFolder => format!("Delete empty generic chunk folder for map \"{}\"", map_name),
			Self::DeleteEmptyChunkFolder(chunk_ref) => format!("Delete empty folder for chunk {:?}", chunk_ref),
			Self::CreateChunksFolder(fix) => fix.description(),
			Self::AlignChunkEdges(ref1, ref2) => format!("Modify chunks {:?} and {:?} so their edges line up", ref1, ref2)
		}
	}
	fn fix(&self, map_name: &str, chunk_size: UInt) -> Result<(), String> {
		match self {
			Self::DeleteEmptyGenericChunkFolder => to_string_err(fs::remove_dir(ChunkRef{position: IntV2(0, 0)}.resource_dir(map_name, true))),
			Self::DeleteEmptyChunkFolder(chunk_ref) => to_string_err(fs::remove_dir(chunk_ref.resource_dir(map_name, false))),
			Self::CreateChunksFolder(fix) => fix.fix(),
			Self::AlignChunkEdges(ref1, ref2) => {
				// Assert that they are adjacent
				let diff: IntV2 = ref2.position - ref1.position;
				// TODO
				Ok(())
			}
		}
	}
}

/// Map struct used by the server, has non-Serialize/Deserialize-able fields
#[cfg(feature = "server")]
pub struct ServerMap {
	pub generic: GenericMap,
	pub path_set: PathSet,
	gen: MapGenerator,
	chunk_creator: ChunkCreationManager
}

#[cfg(feature = "server")]
impl ServerMap {
	/// Creates a new `Self` from a save map. It will return an error if the path set cannot be created from save.
	pub fn from_save(save: SaveMap) -> Result<Self, String> {
		Ok(Self {
			generic: save.generic,
			path_set: PathSet::from_save(save.path_set)?,
			gen: save.gen,
			chunk_creator: ChunkCreationManager::new(save.chunk_creation_rate_limit, save.active_chunk_creators_limit)
		})
	}
	pub fn new(name: &str, chunk_size: UInt, chunk_grid_size: UInt, gen: MapGenerator, background_color: [u8; 3]) -> Self {
		Self {
			generic: GenericMap::new(
				name,
				chunk_size,
				chunk_grid_size,
				background_color
			),
			path_set: PathSet::default(),
			gen,
			chunk_creator: ChunkCreationManager::new(4.0, 20)// TODO: fix arbitrary value
		}
	}
	pub fn send(&self, #[cfg(feature = "debug_render_physics")] physics: &mut PhysicsStateSend) -> SendMap {
		// Unloads all chunks from the physics state which will also be sent to the client
		let mut out_generic = self.generic.clone();
		#[cfg(feature = "debug_render_physics")]
		out_generic.unload_chunks(&Vec::<ChunkRef>::new(), &mut physics.build_body_creation_deletion_context());
		// Done
		SendMap {
			generic: out_generic,
			path_set: self.path_set.clone()
		}
	}
	pub fn load_or_create_chunk(&mut self, ref_: &ChunkRef) -> Option<Chunk> {
		return if ref_.exists(&self.generic.name) {// Chunk already exists on the disk, load it
			//println!("Loading chunk {:?}", &ref_);
			let _filesystem_access = self.chunk_creator.filesystem_lock.lock().unwrap();
			Some(Chunk::load(ref_, &self.generic.name, false).unwrap())
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
		let results = self.chunk_creator.update();
		let mut out = Vec::<Result<(), String>>::new();
		for res in results {
			match res {
				ChunkCreationResult::Ok(chunk) => {self.generic.insert_chunk(chunk, Some(rapier_data));},
				ChunkCreationResult::Err(e, chunk_ref) => out.push(Err(format!("Error creating chunk at {}: {}", chunk_ref.resource_dir_name(), e)))
			}
		}
		// Done
		out
	}
	pub fn save(&self) -> SaveMap {
		SaveMap {
			generic: self.generic.clone(),
			gen: self.gen.clone(),
			path_set: self.path_set.save(),
			active_chunk_creators_limit: self.chunk_creator.active_chunk_creators_limit,
			chunk_creation_rate_limit: self.chunk_creator.rate_limit
		}
	}
}

#[cfg(feature = "server")]
struct ChunkCreationManager {
	active_chunk_creators: Vec<AsyncChunkCreator>,
	creation_stack_before_threads: Vec<ChunkCreationArgs>,// For chunks to be created after the `active_chunk_creators` length goes below `active_chunk_creators_limit`
	active_chunk_creators_limit: usize,// Maximum length of `active_chunk_creators`
	rate_limit: Float,
	most_recent_creation: Arc<Mutex<Instant>>,
	priority: Arc<Mutex<Vec<ChunkRef>>>,// 0 is highest
	pub filesystem_lock: Arc<Mutex<()>>// To prevent concurrent writing and reading which messes with stuff and also I'm paranoid
}

#[cfg(feature = "server")]
impl ChunkCreationManager {
	pub fn new(rate_limit: Float, active_chunk_creators_limit: usize) -> Self {
		Self {
			active_chunk_creators: Vec::new(),
			creation_stack_before_threads: Vec::new(),
			active_chunk_creators_limit,
			rate_limit,
			most_recent_creation: Arc::new(Mutex::new(Instant::now())),
			priority: Arc::new(Mutex::new(Vec::new())),
			filesystem_lock: Arc::new(Mutex::new(()))
		}
	}
	pub fn start(&mut self, chunk_creation_args: ChunkCreationArgs) {
		// Check if chunk ref is already in priority vec or `creation_stack_before_threads`
		{
			let mut priority_access = self.priority.lock().unwrap();
			if priority_access.contains(&chunk_creation_args.ref_) || self.creation_stack_before_threads.iter().find(|args| {&args.ref_ == &chunk_creation_args.ref_}).is_some() {
				return;
			}
			priority_access.push(chunk_creation_args.ref_.clone());
		}
		if self.active_chunk_creators_limit - self.active_chunk_creators.len() > 0 {
			self.spawn_chunk_creator(chunk_creation_args);
		}
		else {
			self.creation_stack_before_threads.push(chunk_creation_args);
		}
	}
	fn spawn_chunk_creator(&mut self, chunk_creation_args: ChunkCreationArgs) {
		// Clone Arcs and start chunk creator
		let most_recent_creation_clone = self.most_recent_creation.clone();
		let priority_clone = self.priority.clone();
		self.active_chunk_creators.push(AsyncChunkCreator::start(chunk_creation_args, most_recent_creation_clone, priority_clone, self.rate_limit, self.filesystem_lock.clone()));
	}
	pub fn update(&mut self) -> Vec<ChunkCreationResult> {
		// Check chunk creators
		let mut indices_to_delete = Vec::<usize>::new();
		let mut out = Vec::<ChunkCreationResult>::new();
		for (i, mut chunk_creator) in &mut self.active_chunk_creators.iter_mut().enumerate() {
			let opt = chunk_creator.check();
			match opt {
				Some(result) => {
					out.push(result);
					indices_to_delete.push(i);
				},
				None => {}
			}
		}
		// Iterate over indices to delete
		for i_to_delete in indices_to_delete.iter().rev() {
			assert!(self.active_chunk_creators[*i_to_delete].is_done(), "Chunk creator to delete is not done");
			self.active_chunk_creators.remove(*i_to_delete);
		}
		// Spawn new chunk creators if `self.active_chunk_creators_limit` allows it
		for _ in 0..((self.active_chunk_creators_limit - self.active_chunk_creators.len()).min(self.creation_stack_before_threads.len())) {
			let new_chunk_args = self.creation_stack_before_threads.remove(0);//.pop().expect("Expected value, if this vec is empty then this loop should not even be run");// If this panics then I have messed up the math in the for-loop
			self.spawn_chunk_creator(new_chunk_args);
		}
		// Done
		out
	}
	pub fn remove_chunk_ref_from_priority(priority: &mut Vec<ChunkRef>, chunk_ref: &ChunkRef) {
		// Check priority queue vec
		let i_opt = priority.iter().position(|r| r == chunk_ref);
		match i_opt {
			Some(i) => {priority.remove(i);},
			None => panic!("Couldn't find chunk ref {:?} to remove in priority vec", &chunk_ref)
		}
	}
}

#[cfg(feature = "server")]
struct AsyncChunkCreator {
	start_time: Instant,
	thread_handle_opt: Option<thread::JoinHandle<()>>,
	chunk_ref: ChunkRef,
	result: Arc<Mutex<Option<ChunkCreationResult>>>
}

#[cfg(feature = "server")]
impl AsyncChunkCreator {
	pub fn start(chunk_creation_args: ChunkCreationArgs, most_recent_creation: Arc<Mutex<Instant>>, priority: Arc<Mutex<Vec<ChunkRef>>>, rate_limit: Float, filesystem_lock: Arc<Mutex<()>>) -> Self {
		let result: Arc<Mutex<Option<ChunkCreationResult>>> = Arc::new(Mutex::new(None));
		let result_clone = result.clone();
		let chunk_ref = chunk_creation_args.ref_.clone();
		let thread_handle = thread::Builder::new().name(format!("create-chunk-{}", chunk_ref.resource_dir_name())).spawn(
			move || Self::main_loop(priority, rate_limit, result_clone, most_recent_creation, chunk_creation_args, filesystem_lock)
		).unwrap();
		Self {
			start_time: Instant::now(),
			thread_handle_opt: Some(thread_handle),
			chunk_ref,
			result
		}
	}
	fn main_loop(priority: Arc<Mutex<Vec<ChunkRef>>>, rate_limit: Float, result_mutex: Arc<Mutex<Option<ChunkCreationResult>>>, most_recent_creation: Arc<Mutex<Instant>>, chunk_creation_args: ChunkCreationArgs, filesystem_lock: Arc<Mutex<()>>) {
		// Delay until this is at index 0 of the priority vec
		loop {
			let break_after_sleep = priority.lock().unwrap()[0] == chunk_creation_args.ref_;
			let rate_limit_dur = Duration::from_secs_f32(1.0 / rate_limit);
			let dur_since_last_creation = Instant::now().duration_since(*most_recent_creation.lock().unwrap());
			if rate_limit_dur > dur_since_last_creation {
				thread::sleep(rate_limit_dur - dur_since_last_creation);
			}
			if break_after_sleep {
				break;
			}
		}
		// Create chunk
		*most_recent_creation.lock().unwrap() = Instant::now();
		let result = Chunk::new(chunk_creation_args, filesystem_lock);
		// Remove from priority when done, even if failed
		ChunkCreationManager::remove_chunk_ref_from_priority(&mut priority.lock().unwrap(), result.chunk_ref());
		// Set result
		*result_mutex.lock().unwrap() = Some(result);
	}
	pub fn check(&mut self) -> Option<ChunkCreationResult> {
		if !self.is_done() {// If there is a thread handle, the thread hasn't been joined yet
			let mut result_ref = self.result.lock().unwrap();
			let chunk_res_opt: Option<ChunkCreationResult> = mem::replace(&mut *result_ref, None);
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