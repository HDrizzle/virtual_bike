/* Added 2023-11-6
Paths will represent stuff like roads and trails. Each path wil be defined by a cubic bezier spline.

This will use bezier splines (wiki: https://en.wikipedia.org/wiki/Composite_B%C3%A9zier_curve, cool video: https://youtu.be/jvPPXbo87ds?si=iO_3PqdXF1xfLaTZ&t=874), with the tangent points mirrored
Technically there are two tangent points for each knot point on a spline, but the tangent points are mirrored so only 1 per knot point is needed
*/

use std::{collections::HashMap, sync::Arc, f32::consts::PI};
use nalgebra::UnitQuaternion;
use serde::{Serialize, Deserialize};
#[cfg(feature = "client")]
use bevy::prelude::*;

use crate::prelude::*;

// CONSTS
const PATH_RENDER_SEGMENT_LENGTH: Float = 4.0;// Arbitrary
pub const BCURVE_LENGTH_ESTIMATION_SEGMENTS: usize = 200;// Arbitrary
const BCURVE_SAMPLE_ORIENTATION_DT: Float = 0.01;// Arbitrary
const BCURVE_BINARY_SEARCH_ITERATIONS: usize = 30;// Arbitrary

// Structs
pub struct BCurveSample {
	pub pos: Iso,
	pub curvature: Float
}

#[derive(PartialEq, Debug)]
pub struct BCurve {// Single cubic bezier curve
	pub knots: [V3; 2],
	pub offsets: [V3; 2]
}

impl BCurve {
	pub fn sample(&self, t: Float) -> BCurveSample {// Uses the polynomial coefficients method from https://youtu.be/jvPPXbo87ds?si=a7YoNKOVflyAVVX5&t=463
		let translation_0 = self.sample_translation(t);
		let translation_1 = self.sample_translation(t + BCURVE_SAMPLE_ORIENTATION_DT);
		let diff = translation_0 - translation_1;
		// Orientation (no bank angle for now), TODO: set "up" vector
		let orientaton = UnitQuaternion::face_towards(&diff, &V3::new(0.0, 1.0, 0.0));
		// Done
		BCurveSample {
			pos: Iso {
				rotation: orientaton,
				translation: nalgebra::Translation{vector: translation_0}
			},
			curvature: 0.0// TODO
		}
		
	}
	pub fn sample_translation(&self, t: Float) -> V3 {// Uses the polynomial coefficients method from https://youtu.be/jvPPXbo87ds?si=a7YoNKOVflyAVVX5&t=463
		let mut coords = Vec::<Float>::new();
		for i in 0..3 {
			// Points
			let p0 = self.knots[0][i];
			let p1 = self.knots[0][i] + self.offsets[0][i];
			let p2 = self.knots[1][i] + self.offsets[1][i];
			let p3 = self.knots[1][i];
			// Quick calculation
			let t2 = t.powi(2);
			let t3 = t.powi(3);
			// Done
			coords.push(
				p0 +
				t*(-3.*p0 + 3.*p1) +
				t2*(3.*p0 - 6.*p1 + 3.*p2) +
				t3*(-p0 + 3.*p1 - 3.*p2 + p3)
			);
		}
		V3::from_vec(coords)
	}
	pub fn estimate_length(// Estimates length until `t`, crude but good enough, from ChatGPT
		&self,
		t: Float,
		num_segments: usize,
	) -> Float {
		let mut length = 0.0;
		let delta_t = t / (num_segments as Float);
	
		for i in 0..num_segments {
			let t0 = i as Float * delta_t;
			let t1 = (i + 1) as Float * delta_t;
	
			let v0 = self.sample_translation(t0);
			let v1 = self.sample_translation(t1);
			length += v3_dist(v0, v1);
		}
	
		length
	}
	pub fn step_distance(&self, start_t: Float, change: Float) -> (Float, Float) {// From ChatGPT (with modifications)
		// Estimates t value so that the new length would be `self.estimate_length(start_t, BCURVE_LENGTH_ESTIMATION_SEGMENTS)` + `change`. Works even when `change` is negative.
		// Returns: (New t value, change left over if t would be outside the interval [0, 1])
		assert!(0.0 <= start_t && start_t <= 1.0);
		let start = self.estimate_length(start_t, BCURVE_LENGTH_ESTIMATION_SEGMENTS);
		let mut t_min: Float = 0.0;
		let mut t_max: Float = 1.0;
		let mut t: Float = 0.5;

		let target_length = {
			let target_raw = start + change;
			let total_length = self.estimate_length(1.0, BCURVE_LENGTH_ESTIMATION_SEGMENTS);
			if target_raw <= 0.0 {
				return (0.0, target_raw);
			}
			else {
				if target_raw >= total_length {
					return (1.0, target_raw - total_length);
				}
				else {
					target_raw
				}
			}
		};

		// Binary search to find the parameter value (t) corresponding to the desired distance
		for _ in 0..BCURVE_BINARY_SEARCH_ITERATIONS {
			let length = self.estimate_length(t, BCURVE_LENGTH_ESTIMATION_SEGMENTS);

			if length > target_length {
				t_max = t;
			} else {
				t_min = t;
			}

			t = 0.5 * (t_min + t_max);
		}
		(t, 0.0)
	}
}

pub type PathTypeRef = u64;

#[derive(Serialize, Deserialize, Clone)]
pub struct PathType {
	ref_: PathTypeRef,
	name: String,// Non-identifying, example usage: "Dirt path", "Road", or "Highway"
	width: Float
}

impl Default for PathType {
	fn default() -> Self {
		Self {
			ref_: 0,
			name: "Default path type".to_string(),
			width: 10.0
		}
	}
}

pub type PathRef = u64;

#[derive(Serialize, Deserialize, Clone)]
pub struct Path {// TODO: fix: load the path type from seperate file
	pub type_: Arc<PathType>,
	pub ref_: PathRef,
	pub name: String,// Non-identifying, Example usage: road names
	pub knot_points: Vec<P3>,// The spline must pass through these
	pub tangent_offsets: Vec<V3>,// wrt each knot point
	pub loop_: bool
}

impl Path {
	pub fn create_body_state(&self, path_body_state: PathBoundBodyState) -> BodyStateSerialize {
		assert_eq!(path_body_state.path_ref, self.ref_);
		let bcurve: BCurve = self.get_bcurve(path_body_state.pos.latest_point);
		let mut sample: BCurveSample = bcurve.sample(path_body_state.pos.t);
		if !path_body_state.forward {// If backwards, reverse orientation
			sample.pos.rotation *= UnitQuaternion::from_axis_angle(&V3::y_axis(), PI);// TODO: bank angle
		}
		// Linvel vector
		let lin_vel: V3 = sample.pos.rotation.transform_vector(&V3::new(0.0, 0.0, path_body_state.velocity));
		// Done
		BodyStateSerialize {
			position: sample.pos,
			lin_vel,
			path_bound_opt: Some(path_body_state)
		}
	}
	pub fn sample(&self, pos: &PathPosition) -> BCurveSample {
		let bcurve = self.get_bcurve(pos.latest_point);
		bcurve.sample(pos.t)
	}
	pub fn get_bcurve(&self, i_raw: usize) -> BCurve {
		// Returns: (point behind, point infront)
		let all_points_len = self.knot_points.len();
		assert!(i_raw < all_points_len, "BCurve index ({:?}) latest point should be < the length of the spline's knot points", i_raw);
		let (i0, i1): (usize, usize) = if i_raw == all_points_len - 1 {
			(i_raw, 0)// Loop over
		}
		else {
			(i_raw, i_raw + 1)
		};
		let v0: V3 = self.knot_points[i0].coords;
		let v1: V3 = self.knot_points[i1].coords;
		let offset0: V3 = self.tangent_offsets[i0];
		let offset1: V3 = -self.tangent_offsets[i1];// Negative is here for a reason
		BCurve {
			knots: [v0, v1],
			offsets: [offset0, offset1]
		}
	}
	pub fn step_position_by_world_units(&self, pos: &mut PathPosition, step: Float, loop_override_opt: Option<bool>) -> bool {
		// Returns: whether the position looped over to the beginning/end of this path or is being clamped
		//dbg!((&pos, step));
		let loop_ = match loop_override_opt {
			Some(loop_override) => loop_override,
			None => self.loop_
		};
		let mut unapplied_change: f32 = step;
		let mut bcurve_index: usize = pos.latest_point;
		let mut curr_t: f32 = pos.t;
		let mut looped: bool = false;
		loop {
			let bcurve = self.get_bcurve(bcurve_index);
			let (new_t, change_left_over) = bcurve.step_distance(curr_t, unapplied_change);
			curr_t = new_t;
			unapplied_change = change_left_over;
			//dbg!((new_t, change_left_over, bcurve_index, unapplied_change));
			if change_left_over == 0.0 {
				break;
			}
			else {
				let (new_index_raw, updated_t): (Int, Float) = match new_t {
					0.0 => {
						(bcurve_index as Int - 1, 1.0)
					},
					1.0 => {
						(bcurve_index as Int + 1, 0.0)
					},
					_ => panic!("BCurve step position change left over != 0, however the new t-value is not 0 or 1")
				};
				//dbg!((new_index_raw, updated_t));
				let (new_bcurve_index, looped_this_time) = mod_or_clamp(new_index_raw, (self.knot_points.len() - 0) as UInt, loop_);
				//dbg!((new_bcurve_index, looped_this_time));
				bcurve_index = new_bcurve_index as usize;
				looped = looped || looped_this_time;
				if looped && !loop_ {
					break;
				}
				else {
					curr_t = updated_t;
				}
			}
			//dbg!((bcurve_index, curr_t, looped));
		}
		*pos = PathPosition {
			t: curr_t,
			latest_point: bcurve_index
		};
		//dbg!("Done");
		looped
	}
	pub fn end_position(&self) -> PathPosition {
		PathPosition{latest_point: self.knot_points.len() - 1, t: 1.0}
	}
	pub fn update_body(&self, dt: Float, forces: &BodyForces, v_static: &VehicleStatic, state: &mut PathBoundBodyState) {
		let bcurve = self.get_bcurve(state.pos.latest_point);
		let dir_sign = state.direction_sign() as Float;
		// Acceleration: F = m * a, a = F / m
		let acc = forces.lin.z / v_static.mass;
		// Velocity: V += a * dt
		let new_velocity = state.velocity + (acc * dt);
		// Translation: translation += V * dt
		self.step_position_by_world_units(&mut state.pos, new_velocity * dt * dir_sign, None);// Apply vehicle direction
		state.velocity = new_velocity;
	}
	pub fn sideways_gravity_force_component(&self, pos: &PathPosition, v_static: &VehicleStatic, gravity: Float) -> Float {
		/*v_static.mass * gravity * -{
			// Path angle
			self.sample(pos).rotation.euler_angles().0// https://docs.rs/nalgebra/latest/nalgebra/geometry/type.UnitQuaternion.html#method.euler_angles
		}.sin()*/0.0
	}
	#[cfg(feature = "client")]
	pub fn bevy_mesh(&self, texture_len_width_ratio: Float, start: &PathPosition) -> (Mesh, Option<PathPosition>) {
		/* Crates a mesh with UV mapping. The UV coords are intended for an image with Y starting from the top going down and with the "path travel direction" being vertical.
		`texture_len_width_ratio` is the ratio of the height / width of the texture image being used.
		The layout of the mesh will look like:
		~~~~~~~~~~~
		| *       |
		4         | ---
		| *       |  ^
		|    *    |  |
		|       * |  |
		|         3  | `PATH_RENDER_SEGMENT_LENGTH`
		|       * |  |
		|    *    |  |
		| *       |  v
		2         | ---
		| *       |
		|    *    |
		|       * |
		0 ------- 1
		|<------->| self.type_.width
		Returns: The mesh and an option for the next position to use, if this is `None`, it means that whatever loop iterating over this can now stop.
		*/
		let half_width = self.type_.width / 2.0;
		// Get length used by texture
		let texture_len = self.type_.width * texture_len_width_ratio;
		// Function to get edge positions on the road, TODO: verify
		let get_edge_pos = |center_pos: &PathPosition, sideways_offset: Float| -> V3 {
			let center_iso = self.sample(center_pos).pos;
			add_isometries(&center_iso, &Iso{
				translation: nalgebra::Translation{vector: V3::new(sideways_offset, 0.0, 0.0)},
				rotation: UnitQuaternion::identity()
			}).translation.vector
		};
		// Loop over segments of length `PATH_RENDER_SEGMENT_LENGTH`
		let mut basic_mesh: BasicTriMesh = BasicTriMesh::default();
		let mut uv_coords: Vec<[f32; 2]> = Vec::<[f32; 2]>::new();// For mapping the texture onto the mesh
		// Closure to simultaneously add mesh vertex and UV coord
		let add_point = |basic_mesh: &mut BasicTriMesh, uv_coords: &mut Vec<[f32; 2]>, curr_pos: &PathPosition, sideways_offset_sign: i32, progress_along_texture: Float| {
			// Sideqys offset
			let sideways_offset = half_width * (sideways_offset_sign as Float);
			// New vertex
			basic_mesh.vertices.push(P3::from(get_edge_pos(
				&curr_pos,
				sideways_offset
			)));
			// New UV coord
			uv_coords.push([
				(sideways_offset_sign + 1) as Float / 2.0,
				1.0 - progress_along_texture
			]);
		};
		// Closure to add a new triangle from latest points
		let add_triangle = |basic_mesh: &mut BasicTriMesh, sideways_offset_sign: i32| {
			let middle: i32 = basic_mesh.vertices.len() as i32 - 2;
			basic_mesh.indices.push([
				(middle-sideways_offset_sign) as u32,
				middle as u32,
				(middle+sideways_offset_sign) as u32
			]);
		};
		// First vertex, bottom-left of ASCII diagram above
		//basic_mesh.vertices.push(P3::from(get_edge_pos(start, -half_width)));
		add_point(&mut basic_mesh, &mut uv_coords, start, -1, 0.0);
		let mut curr_pos: PathPosition = start.clone();/*self.get_new_position(
			start.latest_point,
			start.t * start_bcurve.length() + PATH_RENDER_SEGMENT_LENGTH / 2.0
		);*/
		let mut i = 0;// even = right side, odd - left side
		let next_pos: Option<PathPosition> = loop {
			// Get sideways offset
			let sideways_offset_sign: i32 = if i % 2 == 0 {
				1
			}
			else {
				-1
			};
			// Append next edge point / UV coord
			let progress_along_texture: Float = texture_len / (i as Float * (PATH_RENDER_SEGMENT_LENGTH / 2.0));
			add_point(&mut basic_mesh, &mut uv_coords, &curr_pos, sideways_offset_sign, progress_along_texture);
			// New triangle from last 3 vertices
			if i >= 1 {
				add_triangle(&mut basic_mesh, sideways_offset_sign);
			}
			// Next pos/i
			i += 1;
			// Check stop conditions
			let looped_over = self.step_position_by_world_units(&mut curr_pos, PATH_RENDER_SEGMENT_LENGTH / 2.0, Some(false));
			/*if looped_over {
				print!("Hit the end of the path while creating chunk, loop should break now");
			}*/
			//dbg!(&curr_pos);
			let end_of_texture = i as Float * (PATH_RENDER_SEGMENT_LENGTH / 2.0) >= texture_len;
			let end = looped_over || end_of_texture;
			// TODO: stop either at end of this path or end of texture
			if end {
				// Last 2 triangles
				add_point(&mut basic_mesh, &mut uv_coords, &curr_pos, -sideways_offset_sign, 1.0);
				add_triangle(&mut basic_mesh, -sideways_offset_sign);
				add_point(&mut basic_mesh, &mut uv_coords, &curr_pos, sideways_offset_sign, 1.0);
				add_triangle(&mut basic_mesh, sideways_offset_sign);
				// Break
				break if !looped_over {
					Some(curr_pos)
				}
				else {
					None
				};
			}
		};
		// Done
		let mut mesh = basic_mesh.build_bevy_mesh();
		mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uv_coords);
		(mesh, next_pos)
	}
	#[cfg(feature = "client")]
	pub fn init_bevy(&self, commands: &mut Commands, meshes:  &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>, asset_server: &AssetServer) {
		// Loop through entire path until done
		let mut curr_pos = PathPosition::default();
		loop {
			// With help from https://github.com/bevyengine/bevy/blob/main/examples/3d/wireframe.rs
			let texture_handle: Handle<Image> = asset_server.load("road_us.png");
			let material_handle = materials.add(StandardMaterial {
				base_color: Color::rgba(0.5, 0.5, 0.5, 1.0),
				base_color_texture: Some(texture_handle),
				alpha_mode: AlphaMode::Opaque,
				unlit: true,
				..default()
			});
			let (mesh, next_pos_opt) = self.bevy_mesh(5.0/*220.0/708.0*/, &curr_pos);// TODO: fix hardcoded value
			//dbg!(&next_pos_opt);
			// Add mesh to meshes and get handle
			let mesh_handle: Handle<Mesh> = meshes.add(mesh);
			// Insert mesh BEFORE break
			commands.spawn(
				PbrBundle {
					mesh: mesh_handle,
					material: material_handle,
					..default()
				}
			);
			// Check next pos and maybe break
			match next_pos_opt {
				Some(next_pos) => {
					curr_pos = next_pos;
				},
				None => break
			}
		}
	}
}

type RouteId = u64;

#[derive(Serialize, Deserialize, Clone)]
pub struct Route {// List of intersections
	name: String,
	intersections: Vec<Intersection>
}

type IntersectionId = u64;

#[derive(Serialize, Deserialize, Clone)]
pub struct Intersection {
	path_points: Vec<(PathRef, PathPosition)>
}

#[derive(Serialize, Deserialize, Clone)]
pub struct PathSet {
	pub paths: HashMap<PathRef, Path>,
	query_grid_scale: UInt,// 0 means no query grid
	#[serde(skip)]
	query_grid: HashMap<IntV2, Vec<PathRef>>,
	intersections: HashMap<IntersectionId, Intersection>,
	routes: HashMap<RouteId, Route>
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
			query_grid: HashMap::new(),
			intersections: HashMap::new(),
			routes: HashMap::new()
		}
	}
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct PathPosition {
	pub latest_point: usize,
	pub t: Float// 0<= this <= 1
}

impl PathPosition {
	pub fn from(i: usize, f: Float) -> Self {
		Self {
			latest_point: i,
			t: f
		}
	}
}

impl Default for PathPosition {
	fn default() -> Self {
		Self {
			latest_point: 0,
			t: 0.0
		}
	}
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct PathBoundBodyState {
	pub path_ref: PathRef,
	pub pos: PathPosition,
	pub velocity: Float,// in world-units / second. Wrt body
	pub forward: bool
}

impl PathBoundBodyState {
	pub fn direction_sign(&self) -> Int {
		match self.forward {true => 1, false => -1}
	}
}

/*
{
                    "path_ref": 0,
					"pos": {
						"latest_point": 0,
                    	"t": 0.5
					},
                    "velocity": 10,
                    "forward": true
                } */
/*
"knot_points": [
                    [-50, 10, 0],
                    [ 50, 10, 0]
                ],
                "tangent_offsets": [
                    [0, 0, -50],
                    [0, 0,  50]
                ]*/