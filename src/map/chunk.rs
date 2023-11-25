// Created 2023-9-29

use std::{rc::Rc, error::Error, collections::HashMap};
use serde::{Serialize, Deserialize};// https://stackoverflow.com/questions/60113832/rust-says-import-is-not-used-and-cant-find-imported-statements-at-the-same-time
#[cfg(feature = "frontend")]
use bevy::{prelude::*, render::mesh::{PrimitiveTopology, Indices}};
// Rapier 3D physics
use rapier3d::prelude::*;

use crate::{prelude::*, resource_interface};

use super::gis;

// Structs
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct RegularElevationMesh {// This stores the map topography data in a grid of Z (verticle) values
	pub precision: Float,// side-length of each square
	pub grid: Vec<Vec<Float>>// elevation = grid[y][x], where x and y are in grid index units
}

impl RegularElevationMesh {
	#[cfg(feature = "backend")]
	pub fn new(size: UInt, grid_size: UInt, gen: &Gen, position: &ChunkRef, map_name: &str) -> Self {
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
		//println!("Checking for chunks adjacent to: {:?}", position);
		let mut adj_chunks: [Option<Rc<Self>>; 4] = [None, None, None, None];// Has to be expanded like this otherwise Copy will need to be implemented
		for (i_raw, offset) in EightWayDir::iter().enumerate() {
			let i = i_raw / 2;
			if !offset.is_straight() {
				continue;
			}
			//dbg!(i);
			// Create chunk ref
			let chunk_ref = ChunkRef{position: position.position + offset.unit_displacement().mult(size as Int), generic: false};
			// Check if it exists
			if chunk_ref.exists(map_name) {
				//dbg!(chunk_ref.clone());
				adj_chunks[i] = Some(Rc::new(resource_interface::load_chunk_data(&chunk_ref, map_name).unwrap().elevation));
			}
		}
		// Done
		gen.create_mesh(size, grid_size, adj_chunks)
	}
	pub fn build_trimesh(&self, offset: &V3) -> (Vec<P3>, Vec<[u32; 3]>) {
		// Build vertices
		let mut vertices = Vec::<P3>::new();
		let coord_matrix_size = self.grid.len();// Number of fence posts, not spaces
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
		(
			vertices,
			indices
		)
	}
	pub fn rapier_collider(&self, offset: &V3) -> Collider {
		// https://docs.rs/rapier3d/0.17.2/rapier3d/geometry/struct.ColliderBuilder.html#method.trimesh
		let (vertices, indices) = self.build_trimesh(offset);
		ColliderBuilder::trimesh(
			vertices,
			indices
		).friction(2.0)
		.restitution(0.9)// TODO: fix const value
		.build()
	}
	pub fn flatten_and_reverse_indices(&self, indices: &Vec<[u32; 3]>) -> Vec<u32> {
		let mut out = Vec::<u32>::new();
		for set in indices {
			out.push(set[0]);
			out.push(set[2]);// Not a mistake
			out.push(set[1]);// Triangle winding or something, do not change
		}
		out
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
	#[cfg(feature = "frontend")]
	pub fn bevy_mesh(&self, size: u64, offset: &V3) -> Mesh {// With help from: https://stackoverflow.com/questions/66677098/how-can-i-manually-create-meshes-in-bevy-with-vertices
		let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
		let (vertices, indices) = self.build_trimesh(offset);

		// Positions of the vertices
		// See https://bevy-cheatbook.github.io/features/coords.html
		let mut vertex_arrays = Vec::<[f32; 3]>::new();
		for v in &vertices {
			vertex_arrays.push([v[0], v[1], v[2]]);
		}
		mesh.insert_attribute(
			Mesh::ATTRIBUTE_POSITION,
			vertex_arrays,
		);

		// In this example, normals and UVs don't matter,
		// so we just use the same value for all of them
		mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0., 1., 0.]; vertices.len()]);
		//mesh.compute_flat_normals();// https://docs.rs/bevy/latest/bevy/render/mesh/struct.Mesh.html#method.compute_flat_normals
		//mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0., 0.]; 3]);
		mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, self.build_uv_coords(size, &vertices, offset));

		// A triangle using vertices 0, 2, and 1.
		// Note: order matters. [0, 1, 2] will be flipped upside down, and you won't see it from behind!
		mesh.set_indices(Some(Indices::U32(self.flatten_and_reverse_indices(&indices))));

		// Done
		mesh
	}
}

/*pub struct SurfaceInfo {
	path_type: Option<String>,
	friction: Float,
	slope: Float,
	slope_dir: Float
}*/

#[derive(Serialize, Deserialize, Clone)]
pub struct Gen {// Based off of this 2D model I built: https://docs.google.com/spreadsheets/d/1gIgTebsoACTb5TGMsN60j4T3h5qrU9yni2nVF5K2rr4/edit#gid=0
	pub seed: u32,
	pub max_slope: Float,
	pub rand_multiplier: Float,
	pub first_sum_multiplier: Float,
	pub second_sum_multiplier: Float,
	pub elev_abs_limit: Float
}

impl Gen {
	// TODO: implement perlin noise, good description at https://www.cs.umd.edu/class/spring2018/cmsc425/Lects/lect13-2d-perlin.pdf
	pub fn create_mesh(&self,
		size: UInt,// Chunk size
		grid_size: UInt,// Chunk grid size
		adj_meshes: [Option<Rc<RegularElevationMesh>>; 4]// >, ^, <, v wrt this chunk, (corresponds with the straight variants of EightWayDir)
	) -> RegularElevationMesh {
		// There are two units being used here: base units and grid index units. base unit * precision = grid unit
		// Grid being built
		let mut grid: Vec<Vec<Float>> = Vec::<Vec<Float>>::new();
		// This is the length of the actual grid matrix
		//let grid_matrix_size_i = (grid_size+1) as Int;
		//let size_i = size as Int;
		// grid_size is spaces inside grid, so coord matrix will be (grid_size + 1)^2
		let precision = (size as Float) / (grid_size as Float);
		// Start `all_known_points`, IMPORTANT: USES GRID INDEX UNITS, NOT GRID PRECISION UNITS
		let mut all_known_points = Self::get_edge_points_from_adj_meshes(grid_size, &adj_meshes);
		// For testing only
		/*for y in -1..1 {
			for x in 0..grid_size + 1 {
				all_known_points.insert(IntP2(x as Int, y), y as Float);
			}
		}*/
		//dbg!(&all_known_points);
		// Generate rows, TODO: start generating from adjacent chunk if there is one
		for y in 0..grid_size + 1 {
			let mut row: Vec<Float> = Vec::<Float>::new();
			let mut row_points = HashMap::<IntP2, Float>::new();// Only add this to `all_known_points` after row is complete to prevent feedback
			for x in 0..grid_size + 1 {
				let curr_point = IntP2(x as Int, y as Int);
				let elev: Float = match all_known_points.get(&curr_point) {
					Some(elev) => *elev,// Pre-existing elevation data, must be from edge of adjacent chunk
					None => {// No pre-existing elevation
						#[cfg(feature = "experimental_terrain_gen")]
						let elev_opt = self.generate_point_experimental(curr_point, precision, &all_known_points);
						#[cfg(not(feature = "experimental_terrain_gen"))]
						let elev_opt = self.generate_point(curr_point, precision, &all_known_points);
						match elev_opt {// TODO: I think there is a more compact way to do this with an Option
							Some(point) => point,
							None => 0.0//rand_unit() as Float * self.rand_multiplier
						}
					}
				};
				row.push(elev);
				row_points.insert(curr_point, elev);
			}
			grid.push(row);
			all_known_points.extend(row_points.iter());
		}
		//println!("Grid with grid_size={}: {:?}", grid_size, &grid);
		RegularElevationMesh {
			precision,
			grid
		}
	}
	pub fn get_edge_points_from_adj_meshes(
		grid_size: UInt,
		adj_meshes: &[Option<Rc<RegularElevationMesh>>; 4]
	) -> HashMap::<IntP2, Float> {
		// There are two units being used here: base units and grid index units. base unit * precision = grid unit
		// This is the length of the actual grid matrix
		let grid_matrix_size_i = (grid_size+1) as Int;
		// grid_size is spaces inside grid, so coord matrix will be (grid_size + 1)^2
		// Start `all_known_points`, IMPORTANT: USES GRID INDEX UNITS, NOT GRID PRECISION UNITS
		let mut all_known_points = HashMap::<IntP2, Float>::new();
		// Get vec of given edge points, TODO: it doesn't work, help
		for (mesh_i, mesh_opt) in adj_meshes.iter().enumerate() {
			//let i_float = (i as Float) * precision;
			match mesh_opt {
				Some(mesh) => {// points along the side of a single chunk
					//dbg!(chunk.ref_.clone());
					// Get offsets
					let unit_offset: IntP2 = EightWayDir::from_num(mesh_i * 2).unwrap().unit_displacement();
					//dbg!(unit_offset);
					let offset: IntP2 = unit_offset.mult(grid_matrix_size_i - 1);// grid matrix edges will meet, so some values will be redundant
					// Get grid coordinate which will be constant and its value, 0=X, 1=Y
					let (const_coord, const_val): (Int, Int) = match mesh_i {
						0 => (0, 0),
						1 => (1, 0),
						2 => (0, grid_matrix_size_i - 1),
						3 => (1, grid_matrix_size_i - 1),
						invalid => panic!("Index when iterating over array which should be of index 4 is {}", invalid)
					};
					//dbg!(const_coord);
					//dbg!(const_val);
					// Iterate along side of adjacent chunk
					for i in 0..grid_matrix_size_i {
						let pos_wrt_adj: IntP2 = match const_coord {
							0 => IntP2(const_val,              i),
							1 => IntP2(i             , const_val),
							invalid => panic!("Const coord can only be 0 or 1, instead it is {}", invalid)
						};
						//dbg!(pos_wrt_adj);
						let pos_wrt_this = pos_wrt_adj + offset;// TODO assert this and _one_in are +x, +y
						//dbg!(pos_wrt_this);
						let pos_wrt_adj_one_in = pos_wrt_adj + unit_offset;// 1 in from edge, used for calculating slope
						//dbg!(pos_wrt_adj_one_in);
						all_known_points.insert(pos_wrt_this              , mesh.grid[pos_wrt_adj.1        as usize][pos_wrt_adj.0        as usize].clone());// edge
						all_known_points.insert(pos_wrt_this + unit_offset, mesh.grid[pos_wrt_adj_one_in.1 as usize][pos_wrt_adj_one_in.0 as usize].clone());// 1 in from edge, used for calculating slope, thread 'main' panicked at 'index out of bounds: the len is 51 but the index is 18446744073709551615', /Users/hward/Documents/rust/virtual_bike/src/map/chunk.rs:290:82
					}
				},
				None => {}
			}
		}
		all_known_points
	}
	pub fn generate_point(&self, query: IntP2, precision: Float, points: &HashMap<IntP2, Float>) -> Option<Float> {// TODO: test
		match Self::avg_slope_towards_point(query, precision, &points) {// TODO: should check for adjacent elevation first
			None => {// No adjacent slopes
				None
			},
			Some(avg_slope) => {// At least one adjacent slope
				let avg_elev = match Self::avg_adj_elev(query, &points) {
					Some(elev) => elev,
					None => panic!("Average adjacent slope was able to be found for point {:?}, but avg adj elev was not, this shouldn't be possible", query)
				};
				let rand = rand_unit() as Float;
				//println!("Random value: {}", rand);
				// TODO: 1 x n grid to test this
				// FINALLY, use the model in the spreadsheet, get value
				Some((
					avg_elev +
					(
						precision *
						(
							avg_slope +
							(
								// Next slope calculation
								((rand - 0.5) * self.rand_multiplier) -
								(avg_slope * self.first_sum_multiplier) -
								(avg_elev * self.second_sum_multiplier).powi(3)
							)
						).clamp(-self.max_slope, self.max_slope)
					)
				).clamp(-self.elev_abs_limit, self.elev_abs_limit))
			}
		}
	}
	pub fn generate_point_experimental(&self, query: IntP2, precision: Float, points: &HashMap<IntP2, Float>) -> Option<Float> {// TODO: test
		let dir = EightWayDir::S;
		match Self::slope_towards_point(query, dir.clone(), precision, &points) {// TODO: should check for adjacent elevation first
			None => {// No adjacent slopes
				None
			},
			Some(avg_slope) => {// At least one adjacent slope
				let avg_elev = match Self::avg_adj_elev(query, &points) {// points.get(&(query + dir.unit_displacement()))
					Some(elev) => elev,
					None => panic!("Average adjacent slope was able to be found for point {:?}, but avg adj elev was not, this shouldn't be possible", query)
				};
				let rand = rand_unit() as Float;
				//println!("Random value: {}", rand);
				// TODO: 1 x n grid to test this
				// FINALLY, use the model in the spreadsheet, get value
				Some((
					avg_elev +
					(
						precision *
						(
							avg_slope +
							(
								// Next slope calculation
								((rand - 0.5) * self.rand_multiplier) -
								(avg_slope * self.first_sum_multiplier) -
								(avg_elev * self.second_sum_multiplier).powi(3)
							)
						).clamp(-self.max_slope, self.max_slope)
					)
				).clamp(-self.elev_abs_limit, self.elev_abs_limit))
			}
		}
	}
	pub fn avg_slope_towards_point(query: IntP2, precision: Float, points: &HashMap<IntP2, Float>) -> Option<Float> {
		let mut sum = 0.0;
		let mut count = 0;
		for dir in EightWayDir::iter() {
			match Self::slope_towards_point(query, dir, precision, &points) {
				Some(slope) => {
					sum += slope;
					count += 1;
				},
				None => {}
			}
		}
		match count {
			0 => None,
			n => Some(sum / (n as Float))
		}
	}
	pub fn slope_towards_point(query: IntP2, direction: EightWayDir, precision: Float, points: &HashMap<IntP2, Float>) -> Option<Float> {
		/*
		gets slope towards a given point in a given direction
		\   v   /
		  \ v /
		> > X < <
		  / ^ \
		/   ^   \
		*/
		let offset = direction.unit_displacement();
		match points.get(&(query + offset)) {
			Some(close_val) => {
				match points.get(&(query + offset.mult(2))) {
					Some(further_val) => {
						let raw_slope = ((close_val - further_val) as Float) / precision;
						Some(if direction.is_straight() {
							raw_slope
						}
						else {// diagonal
							raw_slope / 0.7071067811
						})
					}
					None => None
				}
			},
			None => None
		}
	}
	pub fn avg_adj_elev(query: IntP2, points: &HashMap<IntP2, Float>) -> Option<Float> {
		// Gets average elevation of points adjacent to `query`
		let mut elevs = Vec::<Float>::new();
		for dir in EightWayDir::iter() {
			match points.get(&(query + dir.unit_displacement())) {
				Some(elev) => elevs.push(*elev),
				None => {}
			}
		}
		match elevs.len() {
			0 => {// No adjacent slopes, random value
				None
			}
			n_elevs => {// At least one data point
				// Get avg elevation
				let mut sum: Float = 0.0;
				for elev in elevs {
					sum += elev;
				}
				Some(sum / (n_elevs as Float))
			}
		}
	}
}

impl Default for Gen {
	fn default() -> Self {
		Self {
			seed: 1,
			max_slope: 10.0,
			rand_multiplier: 1.0,
			first_sum_multiplier: 0.11,
			second_sum_multiplier: 0.02,
			elev_abs_limit: 50.0
		}
	}
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug, Eq, Hash)]
pub struct ChunkRef {// A deterministic reference to a chunk in the world
	pub position: IntP2,
	pub generic: bool
}

impl ChunkRef {
	pub fn from_world_point(p: P3, chunk_size_u: UInt) -> Self {
		let chunk_size_f = chunk_size_u as Float;
		let chunk_size_int = chunk_size_u as Int;
		//println!("Chunk size: {}", chunk_size_f);
		let x = (p[0] / chunk_size_f).floor() as Int * chunk_size_int;
		let y = (p[2] / chunk_size_f).floor() as Int * chunk_size_int;
		Self {
			position: IntP2(x, y),
			generic: false
		}
	}
	pub fn from_chunk_offset_position(pos: V3) -> Self {
		Self {
			position: IntP2(pos[0] as Int, pos[2] as Int),
			generic: false
		}
	}
	pub fn resource_dir_name(&self) -> String {
		assert!(!self.generic, "Should not get resource_dir_name(&self) of ChunkRef which is generic as the result will be meaningless");
		format!("{}_{}", self.position.0, self.position.1)
	}
	pub fn from_resource_dir_name(name: &str) -> Result<Self, String> {
		let parts: Vec<&str> = name.split("_").collect();
		if parts.len() != 2 {
			return Err("Chunk dir name .split(\"_\") length is not 2".to_string());
		}
		let x = to_string_err(parts[0].parse::<Int>())?;
		let y = to_string_err(parts[1].parse::<Int>())?;
		Ok(Self{position: IntP2(x, y), generic: false})
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
			out.push(Self{position: self.position + offset.unit_displacement().mult(size), generic: false});
		}
		out
	}
	pub fn exists(&self, map_name: &str) -> bool {
		resource_interface::find_chunk(&self, map_name).is_ok()
	}
	#[cfg(feature = "backend")]
	pub fn resource_path(&self, map_name: &str, override_generic: Option<bool>) -> String {
		let generic = match override_generic {
			Some(x) => x,
			None => self.generic
		};
		match generic {
			true  => format!("{}{}/generic_chunk/", resource_interface::MAPS_DIR.to_owned(), map_name.to_owned()),
			false => format!("{}{}/chunks/{}/",     resource_interface::MAPS_DIR.to_owned(), map_name.to_owned(), self.resource_dir_name())
		}
	}
}
/*
impl fmt::Debug for ChunkRef {
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
		write!(f, "ChunkRef{{{}, {}}}", self.position[0], self.position[1])
	}
}*/

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
	collider_handle: Option<ColliderHandle>/*,
	#[serde(skip)]
	#[cfg(feature = "frontend")]
	pbr_bundle_entity_commands_opt: Option<EntityCommands>*/
}

impl Chunk {
	pub fn load(ref_: &ChunkRef, map_name: &str) -> Result<Self, Box<dyn Error>> {
		resource_interface::load_chunk_data(ref_, map_name)
	}
	pub fn new(position: V3, size: u64, grid_size: u64, gen: &Gen, background_color: [u8; 3], map_name: &str, map_real_world_location: Option<[Float; 2]>) -> Self {
		let ref_ = ChunkRef::from_chunk_offset_position(position);
		let elevation = match map_real_world_location {
			Some(map_location) => {
				gis::create_mesh_from_real_world(size, grid_size, &ref_, map_location)
			},
			None => RegularElevationMesh::new(size, grid_size, gen, &ref_, map_name)
		};
		let out = Self {
			position: position.clone(),
			ref_: ref_.clone(),
			elevation,
			texture_data: None,
			size,
			grid_size,
			background_color,
			collider_handle: None
		};
		// Save chunk
		//println!("Creating and saving chunk at {}", out.ref_.resource_dir_name());
		resource_interface::save_chunk_data(&out, map_name).unwrap();
		// Done
		out
	}
	pub fn init_rapier(&mut self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, parent_body_handle: RigidBodyHandle) {
		// Make sure this can only be called once
		if let Some(_) = self.collider_handle {
			panic!("init_rapier() called when self.collider_handle is not None, meaning it has been called more than once");
		}
		let collider = self.elevation.rapier_collider(&self.position);
		//collider.set_position(Iso{rotation: UnitQuaternion::identity(), translation: matrix_to_opoint(self.position).into()});
		self.collider_handle = Some(colliders.insert_with_parent(collider, parent_body_handle, bodies));
	}
	pub fn remove_from_rapier(&mut self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, islands: &mut IslandManager) {
		colliders.remove(self.collider_handle.expect("Removing collider from rapier, but self.collider_handle is None"), islands, bodies, false);
		self.collider_handle = None;
	}
	#[cfg(feature = "frontend")]
	pub fn bevy_pbr_bundle(&self, commands: &mut Commands, meshes:  &mut ResMut<Assets<Mesh>>, materials: &mut ResMut<Assets<StandardMaterial>>, asset_server: &AssetServer) {// See https://stackoverflow.com/questions/66677098/how-can-i-manually-create-meshes-in-bevy-with-vertices
		// TODO: paths
		// With help from https://github.com/bevyengine/bevy/blob/main/examples/3d/texture.rs
		let texture_handle: Handle<Image> = asset_server.load("grass_texture_large.png");// TODO: use self.texture_data
		let material_handle = materials.add(StandardMaterial {
			base_color: Color::rgba(0.5, 0.5, 0.5, 1.0),
			base_color_texture: Some(texture_handle),
			alpha_mode: AlphaMode::Blend,
			unlit: true,
			..default()
		});
		let mesh = self.elevation.bevy_mesh(self.grid_size, &self.position);
		commands.spawn(PbrBundle {
			mesh: meshes.add(mesh),
			material: material_handle,
			..default()
		});
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
	#[cfg(feature = "backend")]
	pub fn send(&self, map_name: &str) -> Self {// Makes sure that texture data is loaded
		let mut out = self.clone();
		// Load texture
		let texture_data = resource_interface::load_chunk_texture(&self.ref_, map_name).expect(&format!("Unable to load texture for chunk {:?}", &self.ref_));
		out.texture_data = Some(texture_data);
		out
	}
	pub fn set_position(&mut self, chunk_ref: &ChunkRef) {
		self.ref_ = chunk_ref.clone();
		self.position = ChunkRef::from_chunk_offset_position(self.position).into_chunk_offset_position(self.position[1]);
	}
}