// Seperate file fro map generation, created 2023-12-30

use std::{rc::Rc, collections::HashMap};
use serde::{Serialize, Deserialize};
use crate::prelude::*;

// Trait
pub trait MapGenerator {
	fn create_mesh(&self,
		size: UInt,// Chunk size
		grid_size: UInt,// Chunk grid size
		adj_meshes: [Option<Rc<RegularElevationMesh>>; 4]// >, ^, <, v wrt this chunk, (corresponds with the straight variants of EightWayDir)
	) -> RegularElevationMesh;
	fn get_edge_points_from_adj_meshes(
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
	fn avg_slope_towards_point(query: IntP2, precision: Float, points: &HashMap<IntP2, Float>) -> Option<Float> {
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
	fn slope_towards_point(query: IntP2, direction: EightWayDir, precision: Float, points: &HashMap<IntP2, Float>) -> Option<Float> {
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
	fn avg_adj_elev(query: IntP2, points: &HashMap<IntP2, Float>) -> Option<Float> {
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

// Structs
#[derive(Serialize, Deserialize, Clone)]
pub struct RandomGen {// Based off of this 2D model I built: https://docs.google.com/spreadsheets/d/1gIgTebsoACTb5TGMsN60j4T3h5qrU9yni2nVF5K2rr4/edit#gid=0
	seed: u32,
	max_slope: Float,
	rand_multiplier: Float,
	first_sum_multiplier: Float,
	second_sum_multiplier: Float,
	elev_abs_limit: Float
}

impl RandomGen {
	
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
}

impl MapGenerator for RandomGen {
	// TODO: implement perlin noise, good description at https://www.cs.umd.edu/class/spring2018/cmsc425/Lects/lect13-2d-perlin.pdf
	fn create_mesh(&self,
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
}

impl Default for RandomGen {
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

struct RealTerrainGen {
	pub location: Option<[Float; 2]>
}