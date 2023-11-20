/* Added 2023-11-6
Paths will represent stuff like roads and trails. Each path wil be defined by a cubic bezier spline.

This will use bezier splines (wiki: https://en.wikipedia.org/wiki/Composite_B%C3%A9zier_curve, cool video: https://youtu.be/jvPPXbo87ds?si=iO_3PqdXF1xfLaTZ&t=874), with the tangent points mirrored
Technically there are two tangent points for each knot point on a spline, but the tangent points are mirrored so only 1 per knot point is needed
*/

use nalgebra::{UnitQuaternion, vector};
use serde::{Serialize, Deserialize};

use super::*;
use crate::prelude::*;

// Structs
#[derive(PartialEq, Debug)]
pub struct BCurve {// Single cubic bezier curve, control points are mirrored
	pub offsets: [V3; 2],
	pub knots: [V3; 2]
}

impl BCurve {
	pub fn sample(&self, t: Float) -> Iso {// Uses the polynomial coefficients method from https://youtu.be/jvPPXbo87ds?si=a7YoNKOVflyAVVX5&t=463
		let translation_0 = self.sample_translation(t);
		let translation_1 = self.sample_translation(t + 0.01);// TODO: fix
		let diff = translation_0 - translation_1;
		// Orientation (no bank angle for now), TODO: set "up" vector
		let orientaton = UnitQuaternion::face_towards(&diff, &V3::new(0.0, 1.0, 0.0));
		// Done
		Iso {
			rotation: orientaton,
			translation: nalgebra::Translation{vector: translation_0}
		}
	}
	pub fn sample_translation(&self, t: Float) -> V3 {// Uses the polynomial coefficients method from https://youtu.be/jvPPXbo87ds?si=a7YoNKOVflyAVVX5&t=463
		let mut coords = Vec::<Float>::new();
		for i in 0..3 {
			// Points
			let P0 = self.knots[0][i] + self.offsets[0][i];
			let P1 = self.knots[0][i];
			let P2 = self.knots[1][i];
			let P3 = self.knots[1][i] - self.offsets[1][i];
			// Quick calculation
			let t2 = t.powi(2);
			let t3 = t.powi(3);
			// Done
			coords.push(
				P0 +
				t*(-3.*P0 + 3.*P1) +
				t2*(3.*P0 - 6.*P1 + 3.*P2) +
				t3*(-P0 + 3.*P1 - 3.*P2 + P3)
			);
		}
		V3::from_vec(coords)
	}
	pub fn length(&self) -> Float {
		v3_dist(self.knots[0], self.knots[1])// TODO: fix: https://math.stackexchange.com/questions/2154029/how-to-calculate-a-splines-length-from-its-control-points-and-knots-vector
	}
	pub fn get_real_velocity_mulitplier(&self, t: Float) -> Float {
		self.length()// TODO: fix: veocity is not constant along curve
	}
}

pub type PathTypeRef = u64;

#[derive(Serialize, Deserialize, Clone)]
pub struct PathType {
	ref_: PathTypeRef,
	name: String,// Non-identifying, example usage: "dirt_path", "road", or "highway"
	width: Float
}

pub type PathRef = u64;

#[derive(Serialize, Deserialize, Clone)]
pub struct Path {// Use for things like roads
	pub type_: PathTypeRef,
	pub ref_: PathRef,
	pub name: String,// Non-identifying, Example usage: road names
	pub knot_points: Vec<P3>,// The spline must pass through these
	pub tangent_offsets: Vec<V3>,// wrt each knot point
	pub loop_: bool
}

impl Path {
	pub fn create_body_state(&self, path_body_state: PathBoundBodyState) -> BodyStateSerialize {
		assert_eq!(path_body_state.path_ref, self.ref_);
		let bcurve = self.get_bcurve(&path_body_state);
		let position = bcurve.sample(path_body_state.ratio_from_latest_point);
		let linvel_scalar = bcurve.length() * path_body_state.velocity;// TODO: fix: veocity is not constant along curve
		// Linvel vector
		let linvel: V3 = position.rotation.transform_vector(&V3::new(0.0, 0.0, linvel_scalar));
		// Done
		BodyStateSerialize {
			position,
			lin_vel: linvel,
			path_bound_opt: Some(path_body_state)
		}
	}
	/*pub fn get_position(&self, path_body_state: &PathBoundBodyState) -> Iso {
		assert_eq!(path_body_state.path_ref, self.ref_);
		let bcurve = self.get_bcurve(path_body_state);
		bcurve.sample(path_body_state.ratio_from_latest_point)
	}*/
	pub fn get_bcurve(&self, path_body_state: &PathBoundBodyState) -> BCurve {
		// Returns: (point behind, point infront)
		let i_raw = path_body_state.latest_point;
		let all_points_len = self.knot_points.len();
		let (i0, i1): (usize, usize) = match self.loop_ {
			true => {
				assert!(i_raw < all_points_len, "Path body state latest point should be < the length of the spline's knot points when the path is looping");
				if i_raw == all_points_len - 1 {
					(i_raw, 0)
				}
				else {
					(i_raw, i_raw + 1)
				}
			},
			false => {
				assert!(i_raw - 1 < all_points_len, "Path body state latest point - 1 should be < the length of the spline's knot points when the path is not looping");
				(i_raw, i_raw + 1)
			}
		};
		let v0: V3 = self.knot_points[i0].coords;
		let v1: V3 = self.knot_points[i1].coords;
		let offset0: V3 = self.tangent_offsets[i0];
		let offset1: V3 = self.tangent_offsets[i1];
		BCurve {
			offsets: [offset0, offset1],
			knots: [v0, v1]
		}
	}
	pub fn get_new_index_and_ratio(&self, curr_i: usize, new_segment_progress_unclamped: Float) -> (usize, Float) {
		// TODO: this assumes all segements are the same length, fix
		let progress_int: Int = new_segment_progress_unclamped as Int;// TODO: be sure this always rounds down
		let progress_fraction: Float = new_segment_progress_unclamped - (progress_int as Float);
		let progress_int_total = progress_int + curr_i as Int;
		let new_prev_int = (progress_int_total.rem_euclid(self.knot_points.len() as Int)) as usize;
		(new_prev_int, progress_fraction)
	}
	pub fn get_real_velocity(&self, state: &PathBoundBodyState) -> Float {
		let bcurve = self.get_bcurve(state);
		state.velocity * bcurve.get_real_velocity_mulitplier(state.ratio_from_latest_point)
	}
	pub fn update_body(&self, dt: Float, forces: &mut VehicleLinearForces, v_static: &VehicleStatic, state: &mut PathBoundBodyState) {
		let bcurve = self.get_bcurve(&state);
		// Acceleration: F = m * a, a = F / m
		// TODO: add gravity
		let acc = forces.sum() / v_static.mass;// Standard units
		// Velocity: V += a * dt
		let new_velocity = state.velocity + (acc * dt) * bcurve.get_real_velocity_mulitplier(state.ratio_from_latest_point);// `segment_dist`/sec units
		// Translation: translation += V * dt
		let new_segment_progress_unclamped = state.ratio_from_latest_point + new_velocity * dt;
		let (new_latest_i, new_segment_progress): (usize, Float) = self.get_new_index_and_ratio(state.latest_point, new_segment_progress_unclamped);
		// Done
		state.ratio_from_latest_point = new_segment_progress;
		state.latest_point = new_latest_i;
		state.velocity = new_velocity;
	}
}

#[derive(Serialize, Deserialize, Clone)]
pub struct PathSet {
	paths: HashMap<PathRef, Path>,
	query_grid_scale: UInt,// 0 means no query grid
	#[serde(skip)]
	query_grid: HashMap<IntP2, Vec<PathRef>>
}

impl PathSet {
	pub fn get_with_ref(&self, ref_: &PathRef) -> Option<&Path> {
		self.paths.get(ref_)
	}
}

impl Default for PathSet {
	fn default() -> Self {
		Self {
			paths: HashMap::new(),
			query_grid_scale: 0,
			query_grid: HashMap::new()
		}
	}
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct PathBoundBodyState {
	pub path_ref: PathRef,
	pub latest_point: usize,
	pub ratio_from_latest_point: Float,// 0<= this < 1
	pub velocity: Float,// in current-spline-segment-length-units / second
	pub forward: bool
}

/*
{
                    "path_ref": 0,
                    "latest_point": 0,
                    "ratio_from_latest_point": 0.5,
                    "velocity": 0.1,
                    "forward": true
                } */