use crate::map::path::{PathType, Intersection, Route};
use ntest::timeout;
use super::*;
// Initial states
fn square_loop_non_unit_edges() -> Path {// 10 x 10 square loop
	Path {
		generic: GenericPath {
			name: "test".to_owned(),
			knot_points: vec![
				V3::new( 0.0, 0.0,  0.0),// >
				V3::new(10.0, 0.0,  0.0),// ^
				V3::new(10.0, 0.0, 10.0),// <
				V3::new( 0.0, 0.0, 10.0)//  v
			],
			tangent_offsets: vec![
				V3::new(0.0, 0.0, 0.0),
				V3::new(0.0, 0.0, 0.0),
				V3::new(0.0, 0.0, 0.0),
				V3::new(0.0, 0.0, 0.0)
			],
			loop_: true
		},
		type_: Arc::new(PathType::default())
	}
}
fn initial_path_bound_body_state() -> PathBoundBodyState {
	PathBoundBodyState {
		path_query: GenericQuery::id(0),
		pos: PathPosition {
			latest_point: 0,
			t: 0.0
		},
		velocity: 1.0,
		forward: true,
		route_query_opt: None,
		in_range_prev_intersection_opt: None
	}
}
fn initial_path_bound_body_state_past_end() -> PathBoundBodyState {
	PathBoundBodyState {
		path_query: GenericQuery::id(0),
		pos: PathPosition {
			latest_point: 3,
			t: 0.5
		},
		velocity: 0.2,
		forward: true,
		route_query_opt: None,
		in_range_prev_intersection_opt: None
	}
}
fn vehicle_static() -> VehicleStatic {
	VehicleStatic {
		type_name: "vehicle static name".to_owned(),
		mass: 200.0,
		ctr_g_hight: 0.0,
		drag: 0.0,
		cross_sectional_area: 1.0,
		wheels: Vec::new()
	}
}

fn intersections_initial_state() -> PathSet {
	let path = square_loop_non_unit_edges();
	let mut paths = GenericDataset::<Path>::new();
	paths.items.push((GenericRef::id(0), path.clone()));
	paths.items.push((GenericRef::id(1), path));
	let intersections = GenericDataset::<Intersection> {items: vec![
		(
			GenericRef::id(0),
			Intersection {
				path_points: vec![
					(GenericQuery::<Path>::id(0), PathPosition::new(0, 0.5)),
					(GenericQuery::<Path>::id(1), PathPosition::new(2, 0.5)),
				]
			}
		),
		(
			GenericRef::id(1),
			Intersection {
				path_points: vec![
					(GenericQuery::<Path>::id(0), PathPosition::new(1, 0.5)),
					(GenericQuery::<Path>::id(1), PathPosition::new(3, 0.5)),
				]
			}
		)
	]};
	let routes = GenericDataset::<Route> {items: vec![
		(
			GenericRef::id(0),
			Route {
				name: "test-route".to_string(),
				intersection_decisions: vec![
					(GenericQuery::<Intersection>::id(0), IntersectionDecision::new(1, true)),
					(GenericQuery::<Intersection>::id(1), IntersectionDecision::new(0, true))
				],
				start_path_query: GenericQuery::<Path>::id(0),
				start_path_pos: PathPosition::new(0, 0.0),
				start_forward: true
			}
		)
	]};
	// Done
	PathSet {
		generic: GenericPathSet {
			query_grid_scale: 0,
			intersections,
			routes
		},
		paths,
		query_grid: HashMap::new()
	}
}

// Tests
#[test]
#[timeout(1000)]
fn forward() {
	let path = square_loop_non_unit_edges();
	let v_static = vehicle_static();
	let mut state = initial_path_bound_body_state();
	// Update
	let dt = 11.0;
	let mut forces = PathBodyForceDescription::default();
	path.generic.update_body(dt, &mut forces, &v_static, &mut state, &None);
	// Compare
	let ideal_new_state = PathBoundBodyState {
		path_query: GenericQuery::id(0),
		pos: PathPosition {
			latest_point: 1,
			t: 0.1
		},// side length is 10, 0.5 side length (5 units) + 1 unit = 0.6 of a side length
		velocity: 1.0,
		forward: true,
		route_query_opt: None,
		in_range_prev_intersection_opt: None
	};
	assert_eq!(state.pos, ideal_new_state.pos);
}
#[test]
#[timeout(1000)]
fn backward() {
	let path = square_loop_non_unit_edges();
	let v_static = vehicle_static();
	let mut state = initial_path_bound_body_state();
	state.velocity = -1.0;
	// Update
	let mut forces = PathBodyForceDescription::default();
	// 1
	path.generic.update_body(5.0, &mut forces, &v_static, &mut state, &None);
	// Compare
	assert_eq!(
		state.pos,
		PathPosition {
			latest_point: 3,
			t: 0.5
		}
	);
	// 2
	path.generic.update_body(7.0, &mut forces, &v_static, &mut state, &None);
	// Compare
	assert_eq!(
		state.pos,
		PathPosition {
			latest_point: 2,
			t: 0.8
		}
	);
}
#[test]
#[timeout(1000)]
fn adj_points() {
	let path = square_loop_non_unit_edges();
	// 0 - 1
	assert_eq!(
		path.generic.get_bcurve(0),
		BCurve::new(
			[
				V3::new(0.0, 0.0, 0.0),
				V3::new(10.0, 0.0,  0.0)
			],
			[V3::zeros(); 2]
		)
	);
	// 3 - 0
	assert_eq!(
		path.generic.get_bcurve(3),
		BCurve::new(
			[
				V3::new( 0.0, 0.0, 10.0),
				V3::new( 0.0, 0.0,  0.0)
			],
			[V3::zeros(); 2]
		)
	);
}
#[test]
#[timeout(1000)]
fn get_bcurve() {
	let path = Path {
		generic: GenericPath {
			name: "test".to_owned(),
			knot_points: vec![
				V3::new( 0.0, 10.0, -100.0),
				V3::new( 0.0, 10.0,    0.0),
				V3::new(50.0, 1.00,  100.0)
			],
			tangent_offsets: vec![
				V3::new( 0.0, 0.0, 0.0),
				V3::new(20.0, 0.0, 0.0),
				V3::new( 0.0, 0.0, 0.0)
			],
			loop_: true
		},
		type_: Arc::new(PathType::default())
	};
	// 1st bcurve
	assert_eq!(
		path.generic.get_bcurve(0),
		BCurve::new(
			[
				V3::new( 0.0, 10.0, -100.0),
				V3::new( 0.0, 10.0,    0.0)
			],
			[
				V3::new(  0.0, 0.0, 0.0),
				V3::new(-20.0, 0.0, 0.0),
			]
		)
	);
	// 2nd bcurve
	assert_eq!(
		path.generic.get_bcurve(1),
		BCurve::new(
			[
				V3::new( 0.0, 10.0,   0.0),
				V3::new(50.0, 1.00, 100.0)
			],
			[
				V3::new(20.0, 0.0, 0.0),
				V3::new( 0.0, 0.0, 0.0)
			]
		)
	);
}
#[test]
#[timeout(1000)]
fn bcurve_length_estimation() {
	let mut bcurve = BCurve::new(
		[
			V3::new( 0.0, 0.0, 0.0),
			V3::new(50.0, 0.0, 0.0)
		],
		[
			V3::new(0.0, 0.0, 0.0),
			V3::new(0.0, 0.0, 0.0)
		]
	);
	assert_relative_eq!(bcurve.estimate_length(1.0, BCURVE_LENGTH_ESTIMATION_SEGMENTS), 50.0, epsilon = EPSILON);
	assert_relative_eq!(bcurve.estimate_length(0.5, BCURVE_LENGTH_ESTIMATION_SEGMENTS), 25.0, epsilon = EPSILON);// only half-way through will work
}
#[test]
#[timeout(1000)]
fn step_position_by_world_units() {
	// Setup
	let path: Path = square_loop_non_unit_edges();
	let mut pos: PathPosition = PathPosition::default();
	// + 5
	assert!(!path.generic.step_position_by_world_units(&mut pos, 5.0, None, &None).0);
	assert_eq!(
		pos,
		PathPosition {
			latest_point: 0,
			t: 0.5
		}
	);
	// + 10 
	assert!(!path.generic.step_position_by_world_units(&mut pos, 10.0, None, &None).0);
	assert_eq!(
		pos,
		PathPosition {
			latest_point: 1,
			t: 0.5
		}
	);
	// - 20
	assert!(path.generic.step_position_by_world_units(&mut pos, -20.0, None, &None).0);
	assert_eq!(
		pos,
		PathPosition {
			latest_point: 3,
			t: 0.5
		}
	);
	// + 40
	assert!(path.generic.step_position_by_world_units(&mut pos, 40.0, None, &None).0);
	assert_eq!(
		pos,
		PathPosition {
			latest_point: 3,
			t: 0.5
		}
	);
}
#[test]
#[timeout(1000)]
fn step_position_by_world_units_non_looping() {
	// Setup
	let mut path: Path = square_loop_non_unit_edges();
	path.generic.loop_ = false;
	let mut pos: PathPosition = PathPosition::default();// 0
	// + 15
	assert!(!path.generic.step_position_by_world_units(&mut pos, 15.0, None, &None).0);
	assert_eq!(
		pos,
		PathPosition {
			latest_point: 1,
			t: 0.5
		}
	);
	// + 30
	assert!(path.generic.step_position_by_world_units(&mut pos, 30.0, None, &None).0);
	assert_eq!(
		pos,
		PathPosition {
			latest_point: 3,
			t: 1.0
		}
	);
	// - 45
	assert!(path.generic.step_position_by_world_units(&mut pos, -45.0, None, &None).0);
	assert_eq!(
		pos,
		PathPosition {
			latest_point: 0,
			t: 0.0
		}
	);
}
#[test]
#[timeout(1000)]
fn next_intersection_on_path() {
	// Initial state
	let path_set: PathSet = intersections_initial_state();
	// Next intersection finding
	assert_eq!(
		path_set.next_intersection_on_path(&GenericQuery::id(0), &PathPosition::new(0, 0.0), true),
		Some((GenericRef::id(0), path_set.generic.intersections.get_item_tuple(&GenericQuery::id(0)).expect("expected Intersection").1, PathPosition::new(0, 0.5), 5.0 as Float))
	);
	assert_eq!(
		path_set.next_intersection_on_path(&GenericQuery::id(1), &PathPosition::new(2, 0.5), true),
		Some((GenericRef::id(0), path_set.generic.intersections.get_item_tuple(&GenericQuery::id(0)).expect("expected Intersection").1, PathPosition::new(2, 0.5), 0.0 as Float))
	);
	assert_eq!(
		path_set.next_intersection_on_path(&GenericQuery::id(1), &PathPosition::new(0, 0.5), false),
		Some((GenericRef::id(1), path_set.generic.intersections.get_item_tuple(&GenericQuery::id(1)).expect("expected Intersection").1, PathPosition::new(3, 0.5), 10.0 as Float))
	);
}
#[test]
#[timeout(1000)]
fn step_past_intersection() {
	// Path set
	let path = square_loop_non_unit_edges();
	// Path bound initial state
	let mut pos = PathPosition::new(0, 0.0);
	// Forward, no edge cases
	assert_eq!(
		path.generic.step_position_by_world_units(&mut pos, 5.0, None, &Some(PathPosition::new(0, 0.75))),// Should miss this
		(false, None)
	);
	assert_eq!(pos, PathPosition::new(0, 0.5));
	assert_eq!(
		path.generic.step_position_by_world_units(&mut pos, 15.0, None, &Some(PathPosition::new(1, 0.5))),// Should hit this with 5 left over
		(false, Some(5.0))
	);
	assert_eq!(pos, PathPosition::new(1, 0.5));
	// Backward, no edge cases
	assert_eq!(
		path.generic.step_position_by_world_units(&mut pos, -25.0, None, &Some(PathPosition::new(3, 0.5))),
		(true, Some(-5.0))
	);
	assert_eq!(pos, PathPosition::new(3, 0.5));
	// Backward, edge case
	assert_eq!(
		path.generic.step_position_by_world_units(&mut pos, -10.0, None, &Some(PathPosition::new(2, 1.0))),
		(false, Some(-5.0))
	);
	assert_eq!(pos, PathPosition::new(3, 0.0));
}
#[test]
#[timeout(1000)]
fn route_following() {
	// Initial state
	let path_set = intersections_initial_state();
	let mut path_body_state = PathBoundBodyState {
		path_query: GenericQuery::id(0),
		pos: PathPosition::new(0, 0.0),
		velocity: 5.0,
		forward: true,
		route_query_opt: Some(GenericQuery::<Route>::id(0)),
		in_range_prev_intersection_opt: None
	};
	let v_static = &VehicleStatic{type_name: "test".to_owned(), mass: 100.0, ctr_g_hight: 0.0, drag: 0.0, cross_sectional_area: 1.0, wheels: vec![]};
	// Step
	// TODO: fix
	path_body_state.update(1.0, &PathBodyForceDescription::default(), &v_static, &path_set);
	assert_eq!(path_body_state.path_query, GenericQuery::id(1));
	assert_eq!(path_body_state.pos, PathPosition::new(3, 0.0));
}