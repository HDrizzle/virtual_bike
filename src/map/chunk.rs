// Created 2023-9-29

use std::{rc::Rc, error::Error, collections::HashMap};
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
#[cfg(feature = "client")]
use bevy::prelude::*;
// Rapier 3D physics
#[cfg(any(feature = "server", feature = "debug_render_physics"))]
use rapier3d::prelude::*;

use crate::{prelude::*, resource_interface};

#[cfg(feature = "server")]
use super::gis;

// Structs
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct RegularElevationMesh {// This stores the map topography data in a grid of Z (verticle) values
	pub precision: Float,// side-length of each square
	pub grid: Vec<Vec<Float>>// elevation = grid[y][x], where x and y are in grid index units
}

impl RegularElevationMesh {
	/*#[cfg(feature = "server")]
	pub fn new(size: UInt, grid_size: UInt, gen: &MapGenerator, position: &ChunkRef, map_name: &str) -> Self {
		// TODO: Use gen::Gen instead
		// grid_size is spaces inside grid, so coord matrix will be (grid_size + 1)^2
		/*let mut grid: Vec<Vec<Float>> = Vec::<Vec<Float>>::new();
		for row_i in 0..grid_size + 1 {
			let mut row: Vec<Float> = Vec::<Float>::new();
			for i in 0..grid_size + 1 {
				//row.push((extras::rand_unit() * random_terrain) as Float);// Random
				/*row.push(
					(
						((i as Float / (size[0] as Float)) * 3.14159 * 4.0).cos()
						+((row_i as Float / (size[1] as Float)) * 3.14159 * 4.0).cos()
					)*-2.0
				);*/// cool but not usefull
				//row.push(gen.get_elevation(i as Float, row_i as Float));
				// Mostly flat but with side "walls"
				row.push(
					if row_i == 0 || i == 0 || row_i == grid_size || i == grid_size {
						20.0
					}
					else {
						if i < grid_size / 2 {
							0.0
						}
						else {
							(
								((i as Float / (grid_size as Float)) * 3.14159 * 20.0).cos()
								+((row_i as Float / (grid_size as Float)) * 3.14159 * 20.0).cos()
							)*-2.0
						}
					}
				);
			}
			grid.push(row);
		}
		//println!("Grid with grid_size={}: {:?}", grid_size, &grid);
		Self {
			precision,
			grid
		}*/
		// Get adjacent chunks
		println!("Checking for chunks adjacent to: {:?}", position);
		let mut adj_chunks: [Option<Rc<Self>>; 4] = [None, None, None, None];// Has to be expanded like this otherwise Copy will need to be implemented
		for (i_raw, offset) in EightWayDir::iter().enumerate() {
			let i = i_raw / 2;
			if !offset.is_straight() {
				continue;
			}
			//dbg!(i);
			// Create chunk ref
			let chunk_ref = ChunkRef{position: position.position + offset.unit_displacement().mult(size as Int)};
			// Check if it exists
			if chunk_ref.exists(map_name) {
				//dbg!(chunk_ref.clone());
				adj_chunks[i] = Some(Rc::new(resource_interface::load_chunk_data(&chunk_ref, map_name).unwrap().elevation));
			}
		}
		// Advanced debug
		for i in 0..adj_chunks.len() {
			match &adj_chunks[i] {
				Some(_) => println!("Mesh at index: {}", i),
				None => {}
			}
		}
		// Done
		gen.create_mesh(size, grid_size, adj_chunks)
	}*/
	pub fn build_trimesh(&self, offset: &V3) -> BasicTriMesh {
		// Build vertices, returns (Flattened vec of vertices, Vec of 3-arrays of indices for vertices in the vertices vec which make up each triangle)
		let mut vertices = Vec::<P3>::new();
		let coord_matrix_size = self.grid.len();// Number of "fence posts", not "spaces"
		//let size = (coord_matrix_size - 1) as Float * self.precision;
		for x in 0..coord_matrix_size {
			for z in 0..coord_matrix_size {
				vertices.push(matrix_to_opoint(V3::new(x as Float * self.precision, self.grid[x][z].into(), z as Float * self.precision) + offset));
			}
		}
		// Build triangles
		/*let get_y_between_grid = |x: usize, z: usize| -> Float {
			assert!(z % 2 == 1, "Grid row inter/extrapolation used on wrong row: {}", z);
			// TODO
			return (self.grid[x][z] + (self.grid[x + 1][z] - self.grid[x][z])/2.0) as Float;// interpolate between x and x+1
		};*/
		let get_vertex_index = |x: usize, z: usize| -> u32 {// Gets the `vertices` index corresponding to the grid X and Y coordinates
			to_string_err_with_message(((x * self.grid.len()) + z).try_into(), &format!("x={}, z={}", x, z)).unwrap()// TODO: test
		};
		let mut indices = Vec::<[u32; 3]>::new();
		for z in 0..coord_matrix_size-1 {
			// define the rows to be used to make the bases (|| to the X-axis) and the points that are in between the the grid points
			// edge cases, "half-triangles" on each end
			// TODO
			for x in 0..coord_matrix_size-1 {
				// Simpler one with 2 triangles per square
				if (x+z) % 2 == 0 {// diagonal slants one way
					// 1
					indices.push([
						get_vertex_index(x, z),
						get_vertex_index(x+1, z),
						get_vertex_index(x, z+1)
					]);
					// 2
					indices.push([
						get_vertex_index(x+1, z+1),
						get_vertex_index(x, z+1),
						get_vertex_index(x+1, z)
					]);
				}
				else {// or other way
					// 1
					indices.push([
						get_vertex_index(x, z),
						get_vertex_index(x+1, z+1),
						get_vertex_index(x, z+1)
					]);
					// 2
					indices.push([
						get_vertex_index(x+1, z+1),
						get_vertex_index(x, z),
						get_vertex_index(x+1, z)
					]);
				}
			}
		}
		//println!("Vertices: {:?}, Indices: {:?}", &vertices, &indices);
		// "Skirt" to prevent literal edge cases where rapier can't handle collisions with the edges of triangle meshes
		/*{
			// TODO
			let out: Float = 1.0;
			let down: Float = 1.0;
			let additional_vertices: Vec<P3> = vec![
				point![]
			];
		}*/
		// Done
		BasicTriMesh {
			vertices,
			indices
		}
	}
	#[cfg(any(feature = "server", feature = "debug_render_physics"))]
	pub fn rapier_collider(&self, offset: &V3) -> Collider {
		// https://docs.rs/rapier3d/0.17.2/rapier3d/geometry/struct.ColliderBuilder.html#method.trimesh
		let mesh = self.build_trimesh(offset);
		ColliderBuilder::trimesh(
			mesh.vertices,
			mesh.indices
		).friction(2.0)
		.restitution(0.9)// TODO: fix const value
		.build()
	}
	pub fn build_uv_coords(&self, grid_size: u64, vertices: &Vec<P3>, offset: &V3) -> Vec<[f32; 2]> {
		// Offset is required because unless this is the chunk at (0, 0), the vertex coordinates will be out of the grid precision range
		let mut out = Vec::<[f32; 2]>::new();
		let size = self.precision * (grid_size + 1) as Float;
		for vertex_shifted in vertices {
			let vertex = vertex_shifted - offset;
			out.push([vertex[0] / size, vertex[2] / size]);
		}
		out
	}
	#[cfg(feature = "client")]
	pub fn bevy_mesh(&self, size: u64, offset: &V3) -> Mesh {// With help from: https://stackoverflow.com/questions/66677098/how-can-i-manually-create-meshes-in-bevy-with-vertices
		// If BasicTriMesh::build_bevy_mesh() works, this whole thing can be deleted
		let tri_mesh = self.build_trimesh(offset);
		let mut mesh = tri_mesh.build_bevy_mesh();
		mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, self.build_uv_coords(size, &tri_mesh.vertices, offset));

		// Done
		mesh
	}
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug, Eq, Hash)]
#[cfg_attr(feature = "client", derive(Component))]
pub struct ChunkRef {// A deterministic reference to a chunk in the world
	pub position: IntP2
}

impl ChunkRef {
	pub fn from_world_point(p: P3, chunk_size_u: UInt) -> Self {
		let chunk_size_f = chunk_size_u as Float;
		let chunk_size_int = chunk_size_u as Int;
		//println!("Chunk size: {}", chunk_size_f);
		let x = (p[0] / chunk_size_f).floor() as Int * chunk_size_int;
		let y = (p[2] / chunk_size_f).floor() as Int * chunk_size_int;
		Self {
			position: IntP2(x, y)
		}
	}
	pub fn from_chunk_offset_position(pos: V3) -> Self {
		Self {
			position: IntP2(pos[0] as Int, pos[2] as Int)
		}
	}
	pub fn resource_dir_name(&self) -> String {
		format!("{}_{}", self.position.0, self.position.1)
	}
	pub fn from_resource_dir_name(name: &str) -> Result<Self, String> {
		let parts: Vec<&str> = name.split("_").collect();
		if parts.len() != 2 {
			return Err("Chunk dir name .split(\"_\") length is not 2".to_string());
		}
		let x = to_string_err(parts[0].parse::<Int>())?;
		let y = to_string_err(parts[1].parse::<Int>())?;
		Ok(Self{position: IntP2(x, y)})
	}
	pub fn into_chunk_offset_position(&self, elev_offset: Float) -> V3 {
		V3::new(self.position.0 as Float, elev_offset, self.position.1 as Float)
	}
	pub fn adjacent_chunks(&self, size_u: u64, exclude_diagonals: bool) -> Vec<Self> {
		let size = size_u as Int;
		let mut out = Vec::<Self>::new();
		for offset in EightWayDir::iter() {
			if exclude_diagonals && !offset.is_straight() {
				continue;
			}
			out.push(Self{position: self.position + offset.unit_displacement().mult(size)});
		}
		out
	}
	#[cfg(feature = "server")]
	pub fn exists(&self, map_name: &str) -> bool {
		resource_interface::find_chunk(&self, map_name).is_ok()
	}
	#[cfg(feature = "server")]
	pub fn resource_path(&self, map_name: &str, generic: bool) -> String {
		match generic {
			true  => format!("{}{}/generic_chunk/", resource_interface::MAPS_DIR.to_owned(), map_name.to_owned()),
			false => format!("{}{}/chunks/{}/",     resource_interface::MAPS_DIR.to_owned(), map_name.to_owned(), self.resource_dir_name())
		}
	}
	#[cfg(feature = "server")]
	pub fn get_adjacent_chunk_meshes(&self, chunk_size: UInt, map_name: &str) -> [Option<Rc<RegularElevationMesh>>; 4] {
		println!("Checking for chunks adjacent to: {:?}", self.position);
		let mut adj_meshes: [Option<Rc<RegularElevationMesh>>; 4] = [None, None, None, None];// Has to be expanded like this otherwise Copy will need to be implemented
		for (i_raw, offset) in EightWayDir::iter().enumerate() {
			let i = i_raw / 2;
			if !offset.is_straight() {
				continue;
			}
			//dbg!(i);
			// Create chunk ref
			let chunk_ref = ChunkRef{position: self.position + offset.unit_displacement().mult(chunk_size as Int)};
			// Check if it exists
			if chunk_ref.exists(map_name) {
				//dbg!(chunk_ref.clone());
				adj_meshes[i] = Some(Rc::new(resource_interface::load_chunk_data(&chunk_ref, map_name).unwrap().elevation));
			}
		}
		// Advanced debug
		for i in 0..adj_meshes.len() {
			match &adj_meshes[i] {
				Some(_) => println!("Mesh at index: {}", i),
				None => {}
			}
		}
		// Done
		adj_meshes
	}
	pub fn origin() -> Self {
		Self {
			position: IntP2(0, 0)
		}
	}
}

#[derive(Serialize, Deserialize, Clone)]
pub struct Chunk {
	pub position: V3,// Y-Value will be elevation offset
	pub ref_: ChunkRef,
	pub elevation: RegularElevationMesh,
	#[serde(skip)]// Texture image data is in a seperate file (.png)
	pub texture_data: Option<Vec<u8>>,
	pub size: UInt,
	pub grid_size: UInt,// Number of spaces inside grid, for example if this is 4 then the elevation grid coordinates should be 5x5, because fenc-post problem
	pub background_color: [u8; 3],
	#[serde(skip)]
	#[cfg(any(feature = "server", feature = "debug_render_physics"))]
	collider_handle: Option<ColliderHandle>,
	#[serde(skip)]
	#[cfg(feature = "client")]
	pub asset_id_opt: Option<AssetId<Mesh>>// Can be used for `remove()`: https://docs.rs/bevy/0.11.0/bevy/asset/struct.Assets.html#method.remove
}

impl Chunk {
	#[cfg(feature = "server")]
	pub fn load(ref_: &ChunkRef, map_name: &str) -> Result<Self, Box<dyn Error>> {
		resource_interface::load_chunk_data(ref_, map_name)
	}
	#[cfg(feature = "server")]
	pub fn new(position: V3, size: u64, grid_size: u64, gen: &MapGenerator, background_color: [u8; 3], map_name: &str) -> Self {
		let ref_ = ChunkRef::from_chunk_offset_position(position);
		let elevation = /*match map_real_world_location {
			Some(map_location) => {
				gis::create_mesh_from_real_world(size, grid_size, &ref_, map_location)
			},
			None => RegularElevationMesh::new(size, grid_size, gen, &ref_, map_name)
		}*/gen.create_mesh(MeshCreationArgs {
			chunk_ref: &ref_,
			size,
			grid_size,
			adj_meshes: ref_.get_adjacent_chunk_meshes(size, map_name)
		});
		let out = Self {
			position: position.clone(),
			ref_: ref_.clone(),
			elevation,
			texture_data: None,
			size,
			grid_size,
			background_color,
			collider_handle: None,
			#[cfg(feature = "client")] asset_id_opt: None
		};
		// Save chunk
		//println!("Creating and saving chunk at {}", out.ref_.resource_dir_name());
		resource_interface::save_chunk_data(&out, map_name).unwrap();
		// Done
		out
	}
	#[cfg(any(feature = "server", feature = "debug_render_physics"))]
	pub fn init_rapier(&mut self, rapier_data: &mut RapierBodyCreationDeletionContext, parent_body_handle: RigidBodyHandle) {
		// Make sure this can only be called once
		if let Some(_) = self.collider_handle {
			panic!("init_rapier() called when self.collider_handle is not None, meaning it has been called more than once");
		}
		let collider = self.elevation.rapier_collider(&self.position);
		//collider.set_position(Iso{rotation: UnitQuaternion::identity(), translation: matrix_to_opoint(self.position).into()});
		self.collider_handle = Some(rapier_data.colliders.insert_with_parent(collider, parent_body_handle, &mut rapier_data.bodies));
	}
	#[cfg(any(feature = "server", feature = "debug_render_physics"))]
	pub fn remove_from_rapier(&mut self, rapier_data: &mut RapierBodyCreationDeletionContext) {
		rapier_data.colliders.remove(self.collider_handle.expect("Removing collider from rapier, but self.collider_handle is None"), &mut rapier_data.islands, &mut rapier_data.bodies, false);
		self.collider_handle = None;
	}
	#[cfg(feature = "client")]
	pub fn bevy_pbr_bundle(&mut self, commands: &mut Commands, meshes:  &mut ResMut<Assets<Mesh>>, materials: &mut ResMut<Assets<StandardMaterial>>, asset_server: &AssetServer) {// See https://stackoverflow.com/questions/66677098/how-can-i-manually-create-meshes-in-bevy-with-vertices
		// TODO: paths
		// With help from https://github.com/bevyengine/bevy/blob/main/examples/3d/texture.rs
		let texture_handle: Handle<Image> = asset_server.load("grass_texture_large.png");// TODO: use self.texture_data
		let material_handle = materials.add(StandardMaterial {
			base_color: Color::rgba(0.5, 0.5, 0.5, 1.0),
			base_color_texture: Some(texture_handle),
			alpha_mode: AlphaMode::Opaque,
			unlit: true,
			..default()
		});
		let mesh = self.elevation.bevy_mesh(self.grid_size, &self.position);
		// Add mesh to meshes and get handle
		let mesh_handle: Handle<Mesh> = meshes.add(mesh);
		self.asset_id_opt = Some(mesh_handle.id());
		// Done
		commands.spawn((
			self.ref_.clone(),
			PbrBundle {
				mesh: mesh_handle,
				material: material_handle,
				..default()
			}
		));
	}
	pub fn distance_to_point(&self, point: &P2) -> Float {
		let v = point.coords;
		let self_pos = v3_to_v2(self.position);
		/*let mut within_coords: [bool; 2] = [false; 2];
		let mut perpindicular_dists: [Float; 2] = [0.0; 2];
		for coord_index in 0..2 {
			perpindicular_dists[coord_index] = {
				let diff = v[coord_index] - self_pos[coord_index];
				if 0.0 <= diff && diff <= self.size {
					0.0
				}
				else {
					if diff <= 0.0 {
						diff.abs()
					}
					else {
						diff.abs() - self.size
					}
				}
			}
		}
		(perpindicular_dists[0].powi(2) + perpindicular_dists[1].powi(2)).sqrt()*/
		// For now using simplified computation using the offset point, so it is compatible with the code in client::chunk_manager::RenderDistance::chunks_to_load()
		let dx: Float = self_pos[0] - v[0];
		let dy: Float = self_pos[1] - v[1];
		(dx.powi(2) + dy.powi(2)).sqrt()
	}
	#[cfg(feature = "server")]
	pub fn send(&self, map_name: &str) -> Self {// Makes sure that texture data is loaded
		let mut out = self.clone();
		// Load texture
		let texture_data = resource_interface::load_chunk_texture(&self.ref_, map_name).expect(&format!("Unable to load texture for chunk {:?}", &self.ref_));
		out.texture_data = Some(texture_data);
		out
	}
	pub fn set_position(&mut self, chunk_ref: &ChunkRef) {
		// Sets chunk's position if it has been loaded as a generic chunk because if so it will likely be incorrect
		self.ref_ = chunk_ref.clone();
		self.position = chunk_ref.into_chunk_offset_position(self.position[1]);
	}
}