/* Added 2023-11-6
Paths will represent stuff like roads and trails. Each path wil be defined by a cubic bezier spline.

This will use bezier splines (wiki: https://en.wikipedia.org/wiki/Composite_B%C3%A9zier_curve, cool video: https://youtu.be/jvPPXbo87ds?si=iO_3PqdXF1xfLaTZ&t=874), with the tangent points mirrored
Technically there are two tangent points for each knot point on a spline, but the tangent points are mirrored so only 1 per knot point is needed
*/

use std::{collections::HashMap, sync::Arc};
use nalgebra::{UnitQuaternion, vector};
use serde::{Serialize, Deserialize};
use bevy::{prelude::*, render::mesh::PrimitiveTopology, pbr::wireframe::{NoWireframe, Wireframe, WireframeColor, WireframeConfig, WireframePlugin}};

//use super::*;
use crate::prelude::*;

// CONSTS
const PATH_RENDER_SEGMENT_LENGTH: Float = 4.0;

// Structs
#[derive(PartialEq, Debug)]
pub struct BCurve {// Single cubic bezier curve
	pub knots: [V3; 2],
	pub offsets: [V3; 2]
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
			let P0 = self.knots[0][i];
			let P1 = self.knots[0][i] + self.offsets[0][i];
			let P2 = self.knots[1][i] + self.offsets[1][i];
			let P3 = self.knots[1][i];
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
		let bcurve = self.get_bcurve(&path_body_state.pos);
		let position = bcurve.sample(path_body_state.pos.ratio_from_latest_point);
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
	pub fn sample(&self, pos: &PathPosition) -> Iso {
		let bcurve = self.get_bcurve(pos);
		bcurve.sample(pos.ratio_from_latest_point)
	}
	pub fn get_bcurve(&self, pos: &PathPosition) -> BCurve {
		// Returns: (point behind, point infront)
		let i_raw = pos.latest_point;
		let all_points_len = self.knot_points.len();
		let (i0, i1): (usize, usize) = match self.loop_ {
			true => {
				assert!(i_raw < all_points_len, "Path position ({:?}) latest point should be < the length of the spline's knot points when the path is looping", pos);
				if i_raw == all_points_len - 1 {
					(i_raw, 0)
				}
				else {
					(i_raw, i_raw + 1)
				}
			},
			false => {
				assert!(i_raw < all_points_len - 1, "Path position ({:?}) latest point should be < the length of the spline's knot points - 1 when the path is not looping", pos);
				(i_raw, i_raw + 1)
			}
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
	pub fn get_new_position(&self, curr_i: usize, new_segment_progress_unclamped: Float) -> (PathPosition, bool) {
		// Returns: (next position, whether new pos has been clamped)
		// TODO: fix: this assumes all segements are the same length
		let progress_int: Int = round_float_towards_neg_inf(new_segment_progress_unclamped);
		let progress_fraction: Float = new_segment_progress_unclamped - (progress_int as Float);
		let progress_int_total = progress_int + curr_i as Int;
		let new_prev_int = (progress_int_total.rem_euclid(self.knot_points.len() as Int)) as usize;
		// Clamp to end if not looping
		let forward = new_segment_progress_unclamped > 0.0;
		if// TODO: test this logic
			(!self.loop_) &&
			(
				( forward && new_prev_int >= self.knot_points.len() - 1) ||
				(!forward && new_prev_int > curr_i)// Wraparound, assuming step is not huge
			)
		{
			return (self.end_position(), true);
		}
		// Done
		(
			PathPosition {
				latest_point: new_prev_int,
				ratio_from_latest_point: progress_fraction
			},
			false
		)
	}
	pub fn step_position_by_world_units(&self, pos: &mut PathPosition, step: Float) -> bool {
		// Returns: whether the position looped over to the beginning of this path or is being clamped
		let old_latest_point = pos.latest_point;
		let bcurve = self.get_bcurve(pos);
		let new_segment_progress_unclamped = pos.ratio_from_latest_point + step / bcurve.length();// TODO: fix: assumes linear motion across bcurve
		let (new_pos, clamped) = self.get_new_position(pos.latest_point, new_segment_progress_unclamped);
		*pos = new_pos;
		// Done
		clamped || (pos.latest_point < old_latest_point)// TODO: fix: assumes a step shorter than the whole path itself
	}
	pub fn end_position(&self) -> PathPosition {
		PathPosition{latest_point: self.knot_points.len() - 1, ratio_from_latest_point: 1.0}
	}
	pub fn get_real_velocity(&self, state: &PathBoundBodyState) -> Float {
		let bcurve = self.get_bcurve(&state.pos);
		state.velocity * bcurve.get_real_velocity_mulitplier(state.pos.ratio_from_latest_point)
	}
	pub fn update_body(&self, dt: Float, forces: &mut VehicleLinearForces, v_static: &VehicleStatic, state: &mut PathBoundBodyState, gravity: Float) {
		let bcurve = self.get_bcurve(&state.pos);
		// Add gravity
		forces.gravity = self.sideways_gravity_force_component(&state.pos, v_static, gravity);
		// Acceleration: F = m * a, a = F / m
		let acc = (forces.sum() / v_static.mass) * match state.forward {true => 1.0, false =>-1.0};// Apply vehicle direction
		// Velocity: V += a * dt
		let new_velocity = state.velocity + (acc * dt) * bcurve.get_real_velocity_mulitplier(state.pos.ratio_from_latest_point);// `segment_dist`/sec units
		// Translation: translation += V * dt
		let new_segment_progress_unclamped = state.pos.ratio_from_latest_point + new_velocity * dt;
		let (new_pos, _) = self.get_new_position(state.pos.latest_point, new_segment_progress_unclamped);
		// Done
		state.pos = new_pos;
		state.velocity = new_velocity;
	}
	pub fn sideways_gravity_force_component(&self, pos: &PathPosition, v_static: &VehicleStatic, gravity: Float) -> Float {
		/*v_static.mass * gravity * -{
			// Path angle
			self.sample(pos).rotation.euler_angles().0// https://docs.rs/nalgebra/latest/nalgebra/geometry/type.UnitQuaternion.html#method.euler_angles
		}.sin()*/0.0
	}
	#[cfg(feature = "frontend")]
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
		let start_bcurve = self.get_bcurve(start);
		// Function to get edge positions on the road, TODO: verify
		let get_edge_pos = |center_pos: &PathPosition, sideways_offset: Float| -> V3 {
			let center_iso = self.sample(center_pos);
			add_isometries(&center_iso, &Iso{
				translation: nalgebra::Translation{vector: V3::new(sideways_offset, 0.0, 0.0)},
				rotation: UnitQuaternion::identity()
			}).translation.vector
		};
		// Loop over segments of length `PATH_RENDER_SEGMENT_LENGTH`
		let mut basic_mesh: BasicTriMesh = BasicTriMesh::default();
		let mut uv_coords: Vec<[f32; 2]> = Vec::<[f32; 2]>::new();// For mapping the texture onto the mesh
		// Closure to simultaneously add mesh vertex and UV coord
		let mut add_point = |basic_mesh: &mut BasicTriMesh, uv_coords: &mut Vec<[f32; 2]>, curr_pos: &PathPosition, sideways_offset_sign: i32, progress_along_texture: Float| {
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
		let mut add_triangle = |basic_mesh: &mut BasicTriMesh, sideways_offset_sign: i32| {
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
			start.ratio_from_latest_point * start_bcurve.length() + PATH_RENDER_SEGMENT_LENGTH / 2.0
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
			let looped_over = self.step_position_by_world_units(&mut curr_pos, PATH_RENDER_SEGMENT_LENGTH / 2.0);
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
	#[cfg(feature = "frontend")]
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
			commands.spawn((
				PbrBundle {
					mesh: mesh_handle,
					material: material_handle,
					..default()
				}
			));
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

#[derive(Serialize, Deserialize, Clone)]
pub struct PathSet {
	pub paths: HashMap<PathRef, Path>,
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
pub struct PathPosition {
	pub latest_point: usize,
	pub ratio_from_latest_point: Float// 0<= this < 1
}

impl PathPosition {
	pub fn from(i: usize, f: Float) -> Self {
		Self {
			latest_point: i,
			ratio_from_latest_point: f
		}
	}
}

impl Default for PathPosition {
	fn default() -> Self {
		Self {
			latest_point: 0,
			ratio_from_latest_point: 0.0
		}
	}
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
pub struct PathBoundBodyState {
	pub path_ref: PathRef,
	pub pos: PathPosition,
	pub velocity: Float,// in current-curve-length-units / second
	pub forward: bool
}

impl PathBoundBodyState {
	pub fn full_step(&mut self) {

	}
	pub fn partial_step(&mut self) {

	}
	pub fn average(&mut self, other: &Self) {
		
	}
}

/*
{
                    "path_ref": 0,
					"pos": {
						"latest_point": 0,
                    	"ratio_from_latest_point": 0.5
					},
                    "velocity": 0.1,
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