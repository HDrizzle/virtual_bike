use std::{collections::HashMap, rc::Rc, sync::Arc, f32::consts::PI};
use approx::assert_relative_eq;
use crate::prelude::*;

pub mod gen {
	use crate::map::map_generation;
	use super::*;
	fn test_points() -> HashMap<IntV2, Float> {
		let mut points = HashMap::<IntV2, Float>::new();
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
		points.insert(IntV2(-1, -2), 1.0);
		points.insert(IntV2(0, -2), 3.0);
		points.insert(IntV2(3, -2), 7.0);
		// y = -1
		points.insert(IntV2(-1, -1), 0.0);
		points.insert(IntV2(0, -1), 1.0);
		points.insert(IntV2(1, -1), 1.0);
		points.insert(IntV2(2, -1), 1.0);
		// y = 0
		points.insert(IntV2(-1, 0), 0.0);
		points.insert(IntV2(0, 0), 0.0);
		points.insert(IntV2(1, 0), 1.0);
		points.insert(IntV2(2, 0), 8.0);
		points.insert(IntV2(3, 0), 5.0);
		// y = 1
		points.insert(IntV2(-1, 1), 6.0);
		points.insert(IntV2(0, 1), 0.0);
		// Done
		points
	}
	// Tests
	#[test]
	fn avg_adj_value() {
		let points = test_points();
		assert_eq!(map_generation::avg_adj_elev(IntV2(3, 0), &points), Some(4.5));
		assert_eq!(map_generation::avg_adj_elev(IntV2(1, 1), &points), Some(9.0 / 4.0));
		assert_eq!(map_generation::avg_adj_elev(IntV2(2, 1), &points), Some(14.0 / 3.0));
		assert_eq!(map_generation::avg_adj_elev(IntV2(-2, 0), &points), Some(2.0));
		assert_eq!(map_generation::avg_adj_elev(IntV2(1, -2), &points), Some(6.0 / 4.0));
		assert_eq!(map_generation::avg_adj_elev(IntV2(2, 2), &points), None);
	}
	#[test]
	fn adj_slope() {
		let points = test_points();
		assert_eq!(map_generation::slope_towards_point(IntV2(2, 1), EightWayDir::S, 1.0, &points), Some(7.0));
		assert_eq!(map_generation::slope_towards_point(IntV2(2, 1), EightWayDir::W, 1.0, &points), None);
		assert_eq!(map_generation::slope_towards_point(IntV2(1, 1), EightWayDir::S, 1.0, &points), Some(0.0));
		assert_eq!(map_generation::slope_towards_point(IntV2(1, 1), EightWayDir::W, 1.0, &points), Some(-6.0));
		assert_eq!(map_generation::slope_towards_point(IntV2(2, -2), EightWayDir::NE, 1.0, &points), None);
	}
	#[test]
	fn avg_adj_slope() {
		let points = test_points();
		assert_eq!(map_generation::avg_slope_towards_point(IntV2(2, 1), 1.0, &points), Some(3.5));
		assert_eq!(map_generation::avg_slope_towards_point(IntV2(1, 1), 1.0, &points), Some(-2.0));
		assert_eq!(map_generation::avg_slope_towards_point(IntV2(3, 1), 1.0, &points), Some(7.0 * (2.0 as Float).sqrt()));
		assert_eq!(map_generation::avg_slope_towards_point(IntV2(2, -2), 1.0, &points), Some((-7.0 + (2.0 as Float).sqrt()) / 2.0));
	}
	#[test]
	fn points_from_adj_meshes_basic_two_sides_1x1() {
		let gen = map_generation::Random::default();
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
		//let points = map_generation::get_edge_points_from_adj_meshes(1, 1, &adj_meshes);
		// Compare
		//dbg!(points);
		// Create mesh
		let new_mesh = gen.create_mesh(MeshCreationArgs{chunk_ref: &ChunkRef::origin(), size: 100, grid_size: 1, adj_meshes});
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
		let gen = map_generation::Random {
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
		//let points = map_generation::get_edge_points_from_adj_meshes(1, 1, &adj_meshes);
		// Compare
		//dbg!(points);
		// Create mesh
		let new_mesh = gen.create_mesh(MeshCreationArgs{chunk_ref: &ChunkRef::origin(), size: 100, grid_size: 2, adj_meshes});
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
		let gen = map_generation::default();
		let mut points = HashMap::<IntV2, Float>::new();
		points.insert(IntV2(-2, 0), 0.0);
		points.insert(IntV2(-1, 0), 0.0);
		// Generate more points
		for x in 0..200 {
			let query = IntV2(x, 0);
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

#[cfg(feature = "server")]
pub mod gis {
	use super::*;
	use crate::map::map_generation::gis::*;
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
				t: 0.5
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
				t: 0.5
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
		path.update_body(dt, &mut forces, &v_static, &mut state);
		// Compare
		let ideal_new_state = PathBoundBodyState {
			path_ref: 0,
			pos: PathPosition {
				latest_point: 1,
				t: 0.6367426
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
		path.update_body(dt, &mut forces, &v_static, &mut state);
		// Compare
		assert_eq!(
			state,
			PathBoundBodyState {
				path_ref: 0,
				pos: PathPosition {
					latest_point: 0,
					t: 0.0
				},
				velocity: -5.0,
				forward: true
			}
		);
		// 2
		state.velocity = -7.0;
		path.update_body(dt, &mut forces, &v_static, &mut state);
		// Compare
		assert_eq!(
			state,
			PathBoundBodyState {
				path_ref: 0,
				pos: PathPosition {
					latest_point: 3,
					t: 0.36325753
				},
				velocity: -7.0,
				forward: true
			}
		);
	}
	#[test]
	fn adj_points() {
		let path = square_loop_non_unit_edges();
		// 0 - 1
		assert_eq!(
			path.get_bcurve(initial_path_bound_body_state().pos.latest_point),
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
			path.get_bcurve(initial_path_bound_body_state_past_end().pos.latest_point),
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
			path.get_bcurve(0),
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
			path.get_bcurve(1),
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
	#[test]
	fn bcurve_length_estimation() {
		let bcurve = BCurve {
			knots: [
				V3::new( 0.0, 0.0, 0.0),
				V3::new(50.0, 0.0, 0.0)
			],
			offsets: [
				V3::new(0.0, 0.0, 0.0),
				V3::new(0.0, 0.0, 0.0)
			]
		};
		assert_relative_eq!(bcurve.estimate_length(0.5, BCURVE_LENGTH_ESTIMATION_SEGMENTS), 25.0, epsilon = EPSILON);// only half-way through will work
	}
	#[test]
	fn bcurve_step_distance() {
		let bcurve = BCurve {
			knots: [
				V3::new( 0.0, 0.0, 0.0),
				V3::new(50.0, 0.0, 0.0)
			],
			offsets: [
				V3::new(0.0, 0.0, 0.0),
				V3::new(0.0, 0.0, 0.0)
			]
		};
		assert_eq!(bcurve.step_distance(0.5, 50.0), (1.0, 25.0));
		assert_eq!(bcurve.step_distance(0.5, -50.0), (0.0, -25.0));
		assert_eq!(bcurve.step_distance(0.0, 50.0), (1.0, 0.0));
	}
	#[test]
	fn step_position_by_world_units() {
		// Setup
		let path: Path = square_loop_non_unit_edges();
		let mut pos: PathPosition = PathPosition::default();
		// + 15
		assert!(!path.step_position_by_world_units(&mut pos, 15.0, None));
		assert_eq!(
			pos,
			PathPosition {
				latest_point: 1,
				t: 0.5
			}
		);
		// - 20
		assert!(path.step_position_by_world_units(&mut pos, -20.0, None));
		assert_eq!(
			pos,
			PathPosition {
				latest_point: 3,
				t: 0.5
			}
		);
		// + 40
		assert!(path.step_position_by_world_units(&mut pos, 40.0, None));
		assert_eq!(
			pos,
			PathPosition {
				latest_point: 3,
				t: 0.5
			}
		);
	}
	#[test]
	fn step_position_by_world_units_non_looping() {
		// Setup
		let mut path: Path = square_loop_non_unit_edges();
		path.loop_ = false;
		let mut pos: PathPosition = PathPosition::default();// 0
		// + 15
		assert!(!path.step_position_by_world_units(&mut pos, 15.0, None));
		assert_eq!(
			pos,
			PathPosition {
				latest_point: 1,
				t: 0.5
			}
		);
		// + 30
		assert!(path.step_position_by_world_units(&mut pos, 30.0, None));
		assert_eq!(
			pos,
			PathPosition {
				latest_point: 3,
				t: 1.0
			}
		);
		// - 45
		assert!(path.step_position_by_world_units(&mut pos, -45.0, None));
		assert_eq!(
			pos,
			PathPosition {
				latest_point: 0,
				t: 0.0
			}
		);
	}
}

mod misc {
	use nalgebra::{UnitQuaternion, UnitVector3};
	use super::*;
	#[test]
	fn rounding() {
		assert_eq!(round_float_towards_neg_inf(3.5), 3);
		assert_eq!(round_float_towards_neg_inf(0.0), 0);
		assert_eq!(round_float_towards_neg_inf(-0.0), 0);
		assert_eq!(round_float_towards_neg_inf(-3.0), -3);
		assert_eq!(round_float_towards_neg_inf(-3.5), -4);
	}
	#[test]
	fn fancy_modulus() {
		assert_eq!(mod_or_clamp(5, 10, true), (5, false));
		assert_eq!(mod_or_clamp(5, 10, false), (5, false));
		assert_eq!(mod_or_clamp(0, 10, false), (0, false));
		assert_eq!(mod_or_clamp(9, 10, false), (9, false));
		assert_eq!(mod_or_clamp(10, 10, false), (9, true));
		assert_eq!(mod_or_clamp(-1, 10, false), (0, true));
		assert_eq!(mod_or_clamp(10, 10, true), (0, true));
		assert_eq!(mod_or_clamp(-1, 10, true), (9, true));
	}
	#[test]
	#[should_panic]
	fn rel_eq_test() {
		assert_relative_eq!(EPSILON * 2.0, 0.0, epsilon = EPSILON);// Just to make sure I'm using this right
	}
	#[test]
	fn simple_rotation_composition() {
		let quat = UnitQuaternion::<Float>::from_axis_angle(&V3::x_axis(), PI/4.0);// Quat must not have roll
		assert_relative_eq!(SimpleRotation::from_quat(quat).to_quat(), quat);
	}
	#[test]
	fn simple_rotation_conversion() {
		let quat = UnitQuaternion::<Float>::from_axis_angle(&V3::y_axis(), 0.0);
		assert_eq!(SimpleRotation::from_quat(quat), SimpleRotation{yaw: 0.0, pitch: 0.0});
	}
}