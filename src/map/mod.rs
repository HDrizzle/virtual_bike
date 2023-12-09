// Map main module file

use std::collections::HashMap;
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
#[cfg(feature = "frontend")]
use bevy::{prelude::*, render::mesh::{PrimitiveTopology, Indices}};
#[cfg(feature = "frontend")]
use bevy_rapier3d::plugin::RapierContext;

use crate::{prelude::*, world::PhysicsStateSend};

pub mod chunk;
pub mod path;
#[cfg(feature = "backend")]
pub mod gis;

// Rapier 3D physics
use rapier3d::prelude::*;

//#[cfg(feature = "backend")]

//use nalgebra::{Isometry3, Point3, Point2, Vector3, Vector2, vector, point};

// Structs
#[derive(Serialize, Deserialize, Clone)]
#[cfg_attr(feature = "frontend", derive(Resource))]
pub struct Map {
	pub name: String,
	pub gen: Gen,
	#[serde(skip)]
	pub loaded_chunks: Vec<Chunk>,
	pub path_set: PathSet,
	pub chunk_size: UInt,// length of the side of each chunk
	pub chunk_grid_size: UInt,// Number of points along the side of each chunk
	pub landmarks: HashMap<String, V2>,
	pub background_color: [u8; 3],
	pub auto_gen_chunks: bool,
	#[serde(skip)]
	pub body_handle_opt: Option<RigidBodyHandle>,
	pub real_world_location: Option<[Float; 2]>
}

impl Map {
	pub fn new(name: &str, chunk_size: UInt, chunk_grid_size: UInt, gen: Gen, background_color: [u8; 3]) -> Self {
		Self {
			name: name.to_owned(),
			gen,
			loaded_chunks: Vec::<Chunk>::new(),
			path_set: PathSet::default(),
			chunk_size,
			chunk_grid_size,
			landmarks: HashMap::<String, V2>::new(),
			background_color,
			auto_gen_chunks: true,
			body_handle_opt: None,
			real_world_location: None
		}
	}
	pub fn init_rapier(&mut self, bodies: &mut RigidBodySet) {
		// Create immovable body
		let body = RigidBodyBuilder::kinematic_position_based().position(Iso::identity()).build();
		let body_handle = bodies.insert(body);
		self.body_handle_opt = Some(body_handle);
	}
	#[cfg(feature = "backend")]
	pub fn send(&self, physics: &mut PhysicsStateSend) -> Self {
		// Done
		let mut out = self.clone();
		out.unload_chunks(&Vec::<ChunkRef>::new(), &mut physics.bodies, &mut physics.colliders, &mut physics.islands);
		out
	}
    #[cfg(feature = "frontend")]
	pub fn unload_chunk_client(&mut self, ref_: &ChunkRef, context: &mut RapierContext, commands: &mut Commands, meshes: &mut ResMut<Assets<Mesh>>, materials: &mut ResMut<Assets<StandardMaterial>>) {
		match self.get_chunk_id(ref_) {
			Some(i) => {
				let chunk = &mut self.loaded_chunks[i];
				// Remove Bevy mesh
				meshes.remove(chunk.asset_id_opt.expect("Client-side chunk unloading expects chunks to have Some(HandleId)"));
				// Generic unload
				self.unload_chunk(i, &mut context.bodies, &mut context.colliders, &mut context.islands);
			},
			None => {}
		}
		// TODO
	}
	pub fn unload_chunk(&mut self, i: usize, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, islands: &mut IslandManager) {
		let chunk = &mut self.loaded_chunks[i];
		// Can't simply delete chunk, 1st must remove it from the physics engine
		chunk.remove_from_rapier(bodies, colliders, islands);
		// Now we can delete it
		self.loaded_chunks.remove(i);
	}
	#[cfg(feature = "backend")]
	pub fn unload_chunks(&mut self, chunks_needed: &Vec<ChunkRef>, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, islands: &mut IslandManager) {
		// Unloads all chunks except the ones in `chunks_needed`
        let mut chunks_to_unload = Vec::<usize>::new();
        for (i, chunk) in self.loaded_chunks.iter().enumerate() {
            if !chunks_needed.contains(&chunk.ref_) {
				chunks_to_unload.push(i);
			}
        }
        for i in chunks_to_unload.iter().rev() {// Important to reverse the index order so its from largest to smallest
			//println!("Unloading chunk {:?}", &ref_);
            self.unload_chunk(*i, bodies, colliders, islands);
        }
	}
	#[cfg(feature = "backend")]
	pub fn load_or_create_chunk(&self, ref_: &ChunkRef) -> Option<Chunk> {
		return if ref_.exists(&self.name) {// Chunk already exists on the disk, load it
			//println!("Loading chunk {:?}", &ref_);
			Some(Chunk::load(ref_, &self.name).unwrap())
		}
		else {// Chunk does not exist, create it
			if !self.auto_gen_chunks {
				return None
			}
			Some(Chunk::new(ref_.into_chunk_offset_position(0.0), self.chunk_size, self.chunk_grid_size, &self.gen, self.background_color, &self.name, self.real_world_location))
		};
	}
	pub fn is_chunk_loaded(&self, ref_: &ChunkRef) -> bool {
		for chunk in &self.loaded_chunks {
			if &chunk.ref_ == ref_ {
				return true;
			}
		}
		false
	}
    #[cfg(feature = "frontend")]
	pub fn insert_chunk_client(&mut self, mut chunk: Chunk, rapier_context: &mut RapierContext, commands: &mut Commands, meshes:  &mut ResMut<Assets<Mesh>>, materials: &mut ResMut<Assets<StandardMaterial>>, asset_server: &AssetServer) {
		// Add to bevy rendering world
		if !self.is_chunk_loaded(&chunk.ref_) {
			chunk.bevy_pbr_bundle(commands, meshes, materials, asset_server);
		}
		// Generic insert
		let loaded = self.insert_chunk(chunk, &mut rapier_context.bodies, &mut rapier_context.colliders);
		assert!(loaded, "self.insert_chunk says the chunk was not loaded, however a previous check says the chunk didn't already exist. This should not be possible.");
	}
	pub fn insert_chunk(&mut self, mut chunk: Chunk, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> bool {
		// IMPORTANT: THIS SHOULD BE THE ONLY PLACE WHERE self.loaded_chunks.push() IS CALLED
		if self.is_chunk_loaded(&chunk.ref_) {
			return false;//panic!("Attempt to insert already loaded chunk");
		}
		let body_handle = self.body_handle_opt.expect("Body handle is None when trying to insert chunk, help: maybe init_rapier() hasn\'t been called yet");
		chunk.init_rapier(bodies, colliders, body_handle);
		/*#[cfg(feature = "frontend")]
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
	#[cfg(feature = "frontend")]
	pub fn init_bevy(&mut self, commands: &mut Commands, meshes:  &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>, asset_server: &AssetServer) {
		#[cfg(feature = "path_rendering")]
		for (_, path) in &mut self.path_set.paths.iter_mut() {
			path.init_bevy(commands, meshes, materials, asset_server);
		}
	}
}
