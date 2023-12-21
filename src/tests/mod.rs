use std::{collections::HashMap, rc::Rc, sync::Arc, f64::consts::PI};
use approx::relative_eq;
use crate::prelude::*;

const EPSILON: Float = 1.0e-4;// Arbitrary

pub mod gen {
	use super::*;
	fn test_points() -> HashMap<IntP2, Float> {
		let mut points = HashMap::<IntP2, Float>::new();
		/* Sample data
		6 | 0
		  | v-this is (0, 0)
		0 | 0   1   8   5
		--+--------------
		0 | 1   1   1
		  |
		1 | 3           7
		*/
		// y = -2
		points.insert(IntP2(-1, -2), 1.0);
		points.insert(IntP2(0, -2), 3.0);
		points.insert(IntP2(3, -2), 7.0);
		// y = -1
		points.insert(IntP2(-1, -1), 0.0);
		points.insert(IntP2(0, -1), 1.0);
		points.insert(IntP2(1, -1), 1.0);
		points.insert(IntP2(2, -1), 1.0);
		// y = 0
		points.insert(IntP2(-1, 0), 0.0);
		points.insert(IntP2(0, 0), 0.0);
		points.insert(IntP2(1, 0), 1.0);
		points.insert(IntP2(2, 0), 8.0);
		points.insert(IntP2(3, 0), 5.0);
		// y = 1
		points.insert(IntP2(-1, 1), 6.0);
		points.insert(IntP2(0, 1), 0.0);
		// Done
		points
	}
	// Tests
	#[test]
	fn avg_adj_value() {
		let points = test_points();
		assert_eq!(Gen::avg_adj_elev(IntP2(3, 0), &points), Some(4.5));
		assert_eq!(Gen::avg_adj_elev(IntP2(1, 1), &points), Some(9.0 / 4.0));
		assert_eq!(Gen::avg_adj_elev(IntP2(2, 1), &points), Some(14.0 / 3.0));
		assert_eq!(Gen::avg_adj_elev(IntP2(-2, 0), &points), Some(2.0));
		assert_eq!(Gen::avg_adj_elev(IntP2(1, -2), &points), Some(6.0 / 4.0));
		assert_eq!(Gen::avg_adj_elev(IntP2(2, 2), &points), None);
	}
	#[test]
	fn adj_slope() {
		let points = test_points();
		assert_eq!(Gen::slope_towards_point(IntP2(2, 1), EightWayDir::S, 1.0, &points), Some(7.0));
		assert_eq!(Gen::slope_towards_point(IntP2(2, 1), EightWayDir::W, 1.0, &points), None);
		assert_eq!(Gen::slope_towards_point(IntP2(1, 1), EightWayDir::S, 1.0, &points), Some(0.0));
		assert_eq!(Gen::slope_towards_point(IntP2(1, 1), EightWayDir::W, 1.0, &points), Some(-6.0));
		assert_eq!(Gen::slope_towards_point(IntP2(2, -2), EightWayDir::NE, 1.0, &points), None);
	}
	#[test]
	fn avg_adj_slope() {
		let points = test_points();
		assert_eq!(Gen::avg_slope_towards_point(IntP2(2, 1), 1.0, &points), Some(3.5));
		assert_eq!(Gen::avg_slope_towards_point(IntP2(1, 1), 1.0, &points), Some(-2.0));
		assert_eq!(Gen::avg_slope_towards_point(IntP2(3, 1), 1.0, &points), Some(7.0 * (2.0 as Float).sqrt()));
		assert_eq!(Gen::avg_slope_towards_point(IntP2(2, -2), 1.0, &points), Some((-7.0 + (2.0 as Float).sqrt()) / 2.0));
	}
	#[test]
	fn points_from_adj_meshes_basic_two_sides_1x1() {
		let gen = Gen::default();
		let adj_meshes = [
			Some(Rc::new(RegularElevationMesh {// E
				precision: 100.0,
				grid: vec![
					vec![0.0, 0.0],
					vec![0.0, 0.0]
				]
			})),
			None,// N
			Some(Rc::new(RegularElevationMesh {// W
				precision: 100.0,
				grid: vec![
					vec![1.0, 1.0],
					vec![1.0, 1.0]
				]
			})),
			None// S
		];
		// Get edge points
		//let points = Gen::get_edge_points_from_adj_meshes(1, 1, &adj_meshes);
		// Compare
		//dbg!(points);
		// Create mesh
		let new_mesh = gen.create_mesh(100, 1, adj_meshes);
		let new_mesh_ideal = RegularElevationMesh {
			precision: 100.0,
			grid: vec![
				vec![1.0, 0.0],
				vec![1.0, 0.0]
			]
		};
		// Compare
		assert_eq!(new_mesh, new_mesh_ideal);
	}
	#[test]
	fn points_from_adj_meshes_all_sides_2x2() {
		let precision: Float = 50.0;
		let gen = Gen {
			seed: 1,
			max_slope: 10.0,
			rand_multiplier: 0.0,// Very important to have no randomness for testing
			first_sum_multiplier: 0.0,
			second_sum_multiplier: 0.0,
			elev_abs_limit: 50.0
		};
		let adj_meshes: [Option<Rc<RegularElevationMesh>>; 4] = [// Keep in mind Y is from top to bottum when creating each row
			Some(Rc::new(RegularElevationMesh {// E
				precision,
				grid: vec![
					vec![4.0, 4.0, 4.0],
					vec![4.0, 4.0, 4.0],
					vec![4.0, 4.0, 4.0]
				]
			})),
			Some(Rc::new(RegularElevationMesh {// N
				precision,
				grid: vec![
					vec![6.0, 5.0, 4.0],
					vec![0.0, 0.0, 0.0],
					vec![0.0, 0.0, 0.0]
				]
			})),
			Some(Rc::new(RegularElevationMesh {// W
				precision,
				grid: vec![
					vec![6.0, 6.0, 6.0],
					vec![6.0, 6.0, 6.0],
					vec![6.0, 6.0, 6.0]
				]
			})),
			Some(Rc::new(RegularElevationMesh {// S
				precision,
				grid: vec![
					vec![0.0, 0.0, 0.0],
					vec![0.0, 0.0, 0.0],
					vec![6.0, 5.0, 4.0]
				]
			}))
		];
		// Get edge points
		//let points = Gen::get_edge_points_from_adj_meshes(1, 1, &adj_meshes);
		// Compare
		//dbg!(points);
		// Create mesh
		let new_mesh = gen.create_mesh(100, 2, adj_meshes);
		let new_mesh_ideal = RegularElevationMesh {
			precision,
			grid: vec![
				vec![6.0, 5.0, 4.0],
				vec![6.0, 7.5/* Average adj. value = 5, avg slope = 2.5, so this should = 7.5, inline comments are great*/, 4.0],
				vec![6.0, 5.0, 4.0]
			]
		};
		// Compare
		assert_eq!(new_mesh, new_mesh_ideal);
	}
	/*#[test]
	fn single_line_test() {
		// Creates a 2D terrain profile, should look similar to the 1st graph in https://docs.google.com/spreadsheets/d/1gIgTebsoACTb5TGMsN60j4T3h5qrU9yni2nVF5K2rr4/edit#gid=0
		// All points will be along X-Axis
		// Initial state
		let gen = Gen::default();
		let mut points = HashMap::<IntP2, Float>::new();
		points.insert(IntP2(-2, 0), 0.0);
		points.insert(IntP2(-1, 0), 0.0);
		// Generate more points
		for x in 0..200 {
			let query = IntP2(x, 0);
			match gen.generate_point(query, 1.0, &points) {
				Some(elev) => {
					points.insert(query, elev);
					print!("{},", elev);
				},
				None => println!("Failed to generate new elevation for qurery {:?}", query)
			}
		}
		// Done
		println!("Done");
	}*/
}

pub mod gis {
	// For crate::map::real_world_gen
	use std::f64::consts::E;
	use super::*;
	use crate::map::gis::*;
	#[test]
	fn conversion() {
		relative_eq!(meters_to_degrees((EARTH_RADIUS * 2.0 * (PI as Float)) as Int), 360.0, epsilon = EPSILON);
		assert_eq!(degrees_to_meters(180.0), (EARTH_RADIUS * (PI as Float)) as Int);
	}
	#[test]
	fn composition() {
		assert_eq!(degrees_to_meters(meters_to_degrees(1000)), 1000);
		relative_eq!(meters_to_degrees(degrees_to_meters(1.0)), 1.0, epsilon = EPSILON);
		relative_eq!(meters_to_degrees(degrees_to_meters(180.0)), 180.0, epsilon = EPSILON);
	}
	#[test]
	fn chunk_edge_alignment() {
		let map_anchor = [-69.0, 60.0];
		assert_eq!(
			chunk_local_location(&map_anchor, &ChunkRef{position: IntP2(0, 0)}, IntP2(1000, 1000)),
			chunk_local_location(&map_anchor, &ChunkRef{position: IntP2(1000, 1000)}, IntP2(0, 0))
		)
	}
}

pub mod paths {
	use crate::map::path::PathType;
	use super::*;
	// Initial states
	fn square_loop_non_unit_edges() -> Path {// 10 x 10 square loop
		Path {
			type_: Arc::new(PathType::default()),
			ref_: 0,
			name: "test".to_owned(),
			knot_points: vec![
				P3::new( 0.0, 0.0,  0.0),// >
				P3::new(10.0, 0.0,  0.0),// ^
				P3::new(10.0, 0.0, 10.0),// <
				P3::new( 0.0, 0.0, 10.0)//  v
			],
			tangent_offsets: vec![
				V3::new(0.0, 0.0, 0.0),
				V3::new(0.0, 0.0, 0.0),
				V3::new(0.0, 0.0, 0.0),
				V3::new(0.0, 0.0, 0.0)
			],
			loop_: true
		}
	}
	fn initial_path_bound_body_state() -> PathBoundBodyState {
		PathBoundBodyState {
			path_ref: 0,
			pos: PathPosition {
				latest_point: 0,
				ratio_from_latest_point: 0.5
			},
			velocity: 0.2,
			forward: true
		}
	}
	fn initial_path_bound_body_state_past_end() -> PathBoundBodyState {
		PathBoundBodyState {
			path_ref: 0,
			pos: PathPosition {
				latest_point: 3,
				ratio_from_latest_point: 0.5
			},
			velocity: 0.2,
			forward: true
		}
	}
	fn vehicle_static() -> VehicleStatic {
		VehicleStatic {
			name: "vehicle static name".to_owned(),
			type_: "vehicle static type".to_owned(),
			mass: 200.0,
			ctr_g_hight: 0.0,
			drag: V2::zeros(),
			wheels: Vec::new()
		}
	}
	// Tests
	#[test]
	fn forward() {
		let path = square_loop_non_unit_edges();
		let v_static = vehicle_static();
		let mut state = initial_path_bound_body_state();
		state.velocity = 12.0;
		// Update
		let dt = 1.0;
		let mut forces = BodyForces::default();
		path.update_body(dt, &mut forces, &v_static, &mut state, 0.0);
		// Compare
		let ideal_new_state = PathBoundBodyState {
			path_ref: 0,
			pos: PathPosition {
				latest_point: 1,
				ratio_from_latest_point: 0.70000005
			},// side length is 10, 0.5 side length (5 units) + 1 unit = 0.6 of a side length
			velocity: 12.0,
			forward: true
		};
		assert_eq!(state, ideal_new_state);
	}
	#[test]
	fn backward() {
		let path = square_loop_non_unit_edges();
		let v_static = vehicle_static();
		let mut state = initial_path_bound_body_state();
		// Update
		let dt = 1.0;
		let mut forces = BodyForces::default();
		// 1
		state.velocity = -5.0;
		path.update_body(dt, &mut forces, &v_static, &mut state, 0.0);
		// Compare
		assert_eq!(
			state,
				PathBoundBodyState {
				path_ref: 0,
				pos: PathPosition {
					latest_point: 0,
					ratio_from_latest_point: 0.0
				},
				velocity: -5.0,
				forward: true
			}
		);
		// 2
		state.velocity = -7.0;
		path.update_body(dt, &mut forces, &v_static, &mut state, 0.0);
		// Compare
		assert_eq!(
			state,
				PathBoundBodyState {
				path_ref: 0,
				pos: PathPosition {
					latest_point: 3,
					ratio_from_latest_point: 0.3
				},
				velocity: -7.0,
				forward: true
			}
		);
	}
	#[test]
	fn adj_points() {
		let path = square_loop_non_unit_edges();
		let v_static = vehicle_static();
		// 0 - 1
		assert_eq!(
			path.get_bcurve(&initial_path_bound_body_state().pos),
			BCurve {
				offsets: [V3::zeros(); 2],
				knots: [
					V3::new(0.0, 0.0, 0.0),
					V3::new(10.0, 0.0,  0.0)
				]
			}
		);
		// 3 - 0
		assert_eq!(
			path.get_bcurve(&initial_path_bound_body_state_past_end().pos),
			BCurve {
				offsets: [V3::zeros(); 2],
				knots: [
					V3::new( 0.0, 0.0, 10.0),
					V3::new( 0.0, 0.0,  0.0)
				]
			}
		);
	}
	#[test]
	fn get_new_index_and_ratio() {
		let path = square_loop_non_unit_edges();
		{
			let curr_i: usize = 0;
			let progress_ratio: Float = 1.5;
			assert_eq!(path.get_new_position(curr_i, progress_ratio), (PathPosition::from(1, 0.5), false));
		}
		{
			let curr_i: usize = 3;
			let progress_ratio: Float = 0.5;
			assert_eq!(path.get_new_position(curr_i, progress_ratio), (PathPosition::from(3, 0.5), false));
		}
		{// Wraparound
			let curr_i: usize = 3;
			let progress_ratio: Float = 2.5;
			assert_eq!(path.get_new_position(curr_i, progress_ratio), (PathPosition::from(1, 0.5), false));
		}
	}
	#[test]
	fn get_bcurve() {
		let path = Path {
			type_: Arc::new(PathType::default()),
			ref_: 0,
			name: "test".to_owned(),
			knot_points: vec![
				P3::new( 0.0, 10.0, -100.0),
				P3::new( 0.0, 10.0,    0.0),
				P3::new(50.0, 1.00,  100.0)
			],
			tangent_offsets: vec![
				V3::new( 0.0, 0.0, 0.0),
				V3::new(20.0, 0.0, 0.0),
				V3::new( 0.0, 0.0, 0.0)
			],
			loop_: true
		};
		// 1st bcurve
		assert_eq!(
			path.get_bcurve(&PathPosition { latest_point: 0, ratio_from_latest_point: 0.5 }),
			BCurve {
				knots: [
					V3::new( 0.0, 10.0, -100.0),
					V3::new( 0.0, 10.0,    0.0)
				],
				offsets: [
					V3::new(  0.0, 0.0, 0.0),
					V3::new(-20.0, 0.0, 0.0),
				]
			}
		);
		// 2nd bcurve
		assert_eq!(
			path.get_bcurve(&PathPosition { latest_point: 1, ratio_from_latest_point: 0.5 }),
			BCurve {
				knots: [
					V3::new( 0.0, 10.0,   0.0),
					V3::new(50.0, 1.00, 100.0)
				],
				offsets: [
					V3::new(20.0, 0.0, 0.0),
					V3::new( 0.0, 0.0, 0.0)
				]
			}
		);
	}
}

mod misc {
	use super::*;
	#[test]
	fn rounding() {
		assert_eq!(round_float_towards_neg_inf(3.5), 3);
		assert_eq!(round_float_towards_neg_inf(0.0), 0);
		assert_eq!(round_float_towards_neg_inf(-0.0), 0);
		assert_eq!(round_float_towards_neg_inf(-3.0), -3);
		assert_eq!(round_float_towards_neg_inf(-3.5), -4);
	}
}