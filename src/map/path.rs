//! Added 2023-11-6
//! Paths will represent stuff like roads and trails. Each path wil be defined by a cubic bezier spline.
//! 
//! This uses bezier splines (wiki: https://en.wikipedia.org/wiki/Composite_B%C3%A9zier_curve, cool video: https://youtu.be/jvPPXbo87ds?si=iO_3PqdXF1xfLaTZ&t=874), with the tangent points mirrored
//! Technically there are two tangent points for each knot point on a spline, but the tangent points are mirrored so only 1 per knot point is needed
//! 
//! There are `Route`s which represent a loop and can use multiple paths/parts of paths. A `Path` is a series of cubic bezier curves (`BCurve`).

use std::{collections::HashMap, sync::Arc, f32::consts::PI, net::SocketAddr};
use core::cmp::{PartialOrd, Ordering};
use bevy::math;
use bevy_inspector_egui::egui::epaint::tessellator::path;
use nalgebra::UnitQuaternion;
use serde::{Serialize, Deserialize};
#[cfg(feature = "client")]
use bevy::{prelude::*, render::render_asset::RenderAssetUsages};

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
			length += (v1 - v0).magnitude();
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

pub type PathTypeRef = String;

/// Defines information about any paths of this path type
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PathType {
	/// Reference to this path type
	pub ref_: PathTypeRef,
	/// Non-identifying, example usage: "Dirt path", "Road", or "Highway"
	pub name: String,
	/// Width of path
	pub width: Float
}

impl Default for PathType {
	fn default() -> Self {
		Self {
			ref_: "default-path-type".to_owned(),
			name: "Default path type".to_owned(),
			width: 10.0
		}
	}
}

/// Generic path is Serialize/Deserialize-able and used during runtime, has most of the functionality.
/// Has data which representes a series of cubic beier curves.
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct GenericPath {
	/// Possible identifying name
	pub unique_name_opt: Option<String>,
	/// Non-identifying, Example usage: road names
	pub name: String,
	/// The spline must pass through these points
	pub knot_points: Vec<V3>,
	/// wrt each knot point
	pub tangent_offsets: Vec<V3>,
	/// Whether the end of this path is meant to meet up with the beginning
	pub loop_: bool
}

impl GenericPath {
	pub fn create_body_state(&self, path_body_state: PathBoundBodyState) -> BodyStateSerialize {
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
			ang_vel: V3::zeros(),// TODO
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
		let v0: V3 = self.knot_points[i0];
		let v1: V3 = self.knot_points[i1];
		let offset0: V3 = self.tangent_offsets[i0];
		let offset1: V3 = -self.tangent_offsets[i1];// Negative is here for a reason
		BCurve {
			knots: [v0, v1],
			offsets: [offset0, offset1]
		}
	}
	/// Returns: (
	/// 	whether the position looped over to the beginning/end of this path or is being clamped,
	/// 	Option<Step distance left over past optional given intersection if this step would go past it: Float>
	/// )
	pub fn step_position_by_world_units(
		&self,
		pos: &mut PathPosition,
		step: Float,
		loop_override_opt: Option<bool>,
		next_intersection_opt: Option<Float>
	) -> (bool, Option<Float>) {
		// TODO: use `next_intersection_opt`
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
		(
			looped,
			None// TODO
		)
	}
	/// Estimates the distance along this path to the given path pos using `num_segments` sections per bcurve
	/// ```
	/// use virtual_bike::prelude::*;
	/// use approx::assert_relative_eq;
	/// let path = GenericPath{// Should be a good approximation to the unit circle, https://spencermortensen.com/articles/bezier-circle/
	/// 	unique_name_opt: None,
	/// 	name: "test-path-name".to_owned(),
	/// 	knot_points: vec![
	/// 		V3::new(1.0, 0.0, 0.0),
	/// 		V3::new(0.0, 1.0, 0.0),
	/// 		V3::new(-1.0, 0.0, 0.0),
	/// 		V3::new(0.0, -1.0, 0.0),
	/// 	],
	/// 	tangent_offsets: vec![
	/// 		V3::new(0.0, 0.55342686, 0.0),
	/// 		V3::new(-0.55342686, 0.0, 0.0),
	/// 		V3::new(0.0, -0.55342686, 0.0),
	/// 		V3::new(0.55342686, 0.0, 0.0),
	/// 	],
	/// 	loop_: true
	/// };
	/// assert_relative_eq!(path.estimate_distance_to_position(&path.end_position(), 1000), PI*2.0, epsilon=0.005);// Unit circle = 2*pi
	/// ```
	pub fn estimate_distance_to_position(&self, pos: &PathPosition, num_segments: usize) -> Float {
		let mut out: Float = 0.0;
		for i in 0..pos.latest_point+1 {
			let t = if i == pos.latest_point {
				pos.t
			}
			else {
				1.0
			};
			out += self.get_bcurve(i).estimate_length(t, num_segments);
		}
		// Done
		out
	}
	pub fn end_position(&self) -> PathPosition {
		PathPosition{latest_point: self.knot_points.len() - 1, t: 1.0}
	}
	/// Updates path bound body state. If it goes through an intersection, it will return the remaining step distance after the intersection.
	/// Returns: Optional amount of step distance left after intersection
	pub fn update_body(&self, dt: Float, forces: &BodyForces, v_static: &VehicleStatic, state: &mut PathBoundBodyState, next_intersection_opt: Option<Float>) -> Option<Float> {
		let bcurve = self.get_bcurve(state.pos.latest_point);
		let dir_sign = bool_sign(state.forward) as Float;
		// Acceleration: F = m * a, a = F / m
		let acc = forces.lin.z / v_static.mass;
		// Velocity: V += a * dt
		let new_velocity = state.velocity + (acc * dt);
		// Translation: translation += V * dt
		let total_step = new_velocity * dt * dir_sign;// Apply vehicle direction
		// Set velocity
		state.velocity = new_velocity;
		// Done
		self.step_position_by_world_units(&mut state.pos, total_step, None, next_intersection_opt).1
	}
	pub fn sideways_gravity_force_component(&self, pos: &PathPosition, v_static: &VehicleStatic, gravity: Float) -> Float {
		v_static.mass * gravity * -SimpleRotation::from_quat(&self.sample(&pos).pos.rotation).pitch.sin()
	}
	#[cfg(feature = "client")]
	pub fn bevy_mesh(&self, type_config: &PathType, texture_len_width_ratio: Float, start: &PathPosition) -> (Mesh, Option<PathPosition>) {
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
		let half_width = type_config.width / 2.0;
		// Get length used by texture
		let texture_len = type_config.width * texture_len_width_ratio;
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
			let (looped_over, _) = self.step_position_by_world_units(&mut curr_pos, PATH_RENDER_SEGMENT_LENGTH / 2.0, Some(false), None);
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
		let mut mesh = basic_mesh.build_bevy_mesh(RenderAssetUsages::all());// TODO: confirm
		mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uv_coords);
		(mesh, next_pos)
	}
	#[cfg(feature = "client")]
	pub fn init_bevy(&self, type_config: &PathType, commands: &mut Commands, meshes:  &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>, asset_server: &AssetServer, server_addr: &SocketAddr) {
		// Loop through entire path until done
		let mut curr_pos = PathPosition::default();
		loop {
			// With help from https://github.com/bevyengine/bevy/blob/main/examples/3d/wireframe.rs
			let texture_handle: Handle<Image> = asset_server.load(&format!("http://{:?}/path_textures/{}.png", server_addr, &type_config.ref_));//"road_us.png");
			let material_handle = materials.add(StandardMaterial {
				base_color: Color::rgba(0.5, 0.5, 0.5, 1.0),
				base_color_texture: Some(texture_handle),
				alpha_mode: AlphaMode::Opaque,
				unlit: true,
				..default()
			});
			let (mesh, next_pos_opt) = self.bevy_mesh(type_config, 5.0/*220.0/708.0*/, &curr_pos);// TODO: fix hardcoded value
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

/// This is not loaded/saved but it implements Serialize / Deserialize so it can be sent over to the client
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Path {
	pub generic: GenericPath,
	pub type_: Arc<PathType>
}

impl Path {
	pub fn from_save(save: SavePath, type_: Arc<PathType>) -> Self {
		assert_eq!(save.type_, type_.ref_);
		Self {
			generic: save.generic,
			type_
		}
	}
	#[cfg(feature = "client")]
	pub fn init_bevy(&self, commands: &mut Commands, meshes:  &mut Assets<Mesh>, materials: &mut Assets<StandardMaterial>, asset_server: &AssetServer, server_addr: &SocketAddr) {
		self.generic.init_bevy(&self.type_, commands, meshes, materials, asset_server, server_addr)
	}
	pub fn save(&self) -> SavePath {
		SavePath {
			generic: self.generic.clone(),
			type_: self.type_.ref_.clone()
		}
	}
}

/// Loaded / saved, this struct is different from `Path` because this only has a reference to it's type (string representing name of resource JSON file) and `Path` has an Arc of the actual data.
#[derive(Serialize, Deserialize, Clone)]
pub struct SavePath {
	pub generic: GenericPath,
	pub type_: PathTypeRef
}

/// A Route which will suggest intersection decisions for a vehicle
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct Route {// List of intersections
	/// Non-identifying
	name: String,
	/// Vec of intersection desicions
	pub intersection_decisions: Vec<(GenericQuery<Intersection>, IntersectionDecision)>,
	/// Start
	start: PathBoundBodyState
}

impl Route {
	/// Gets the decision for the first time that this route encounters `IntersectionId` (if at all)
	pub fn unreliable_decision_opt(&self, intersection_query: GenericQuery<Intersection>, forward: bool, intersections: &GenericDataset<Intersection>) -> Option<IntersectionDecision> {
		let int_id = match intersections.get_item_id(&intersection_query) {
			Some(id) => id,
			None => return None
		};
		for decision_t in &self.intersection_decisions {
			if let Some(curr_id) = intersections.get_item_id(&intersection_query) {
				if curr_id == int_id {
					return Some(decision_t.1.clone())
				}
			}
		}
		// Done
		None
	}
}

/// A "decision" for an intersection - represents which way to turn
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct IntersectionDecision {
	pub exit: usize,// Index for intersection.path_points
	pub forward: bool// Whether going forward on new path coming out of intersection
}

impl IntersectionDecision {
	pub fn new(
		exit: usize,
		forward: bool
	) -> Self {
		Self {
			exit,
			forward
		}
	}
}

/// An intersection between paths, all `path_point`s are assumed to be in the same place, or at least very close to each other
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Default)]
pub struct Intersection {
	pub path_points: Vec<(GenericQuery<Path>, PathPosition)>
}

impl Intersection {
	pub fn new(
		path_points: Vec<(GenericQuery<Path>, PathPosition)>
	) -> Self {
		Self {
			path_points
		}
	}
	pub fn default_decision(&self, paths: &PathSet, curr_path: &GenericQuery<Path>) -> IntersectionDecision {
		let mut out = IntersectionDecision {
			exit: 0,
			forward: true
		};
		// TODO
		// Done
		out
	}
	/// Applies intersection decision to `state`
	/// ```
	/// use virtual_bike::prelude::*;
	/// let paths = GenericDataset::<Path> {items: vec![
	/// 	(GenericRef::id(0), Path::default())
	/// ]};
	/// let int = Intersection::new(
	/// 	vec![
	/// 		(GenericQuery::id(0), PathPosition::new(0, 0.5)),
	/// 		(GenericQuery::id(0), PathPosition::new(10, 0.75))
	/// 	]
	/// );
	/// let decision = IntersectionDecision::new(1, true);
	/// let mut state = PathBoundBodyState {
	/// 	path_query: GenericQuery::id(0),
	/// 	pos: PathPosition::new(0, 0.5),
	/// 	velocity: 10.0,
	/// 	forward: true,
	/// 	route_query_opt: None
	/// };
	/// int.apply_decision(&decision, &mut state, &paths);
	/// assert_eq!(state.pos, PathPosition::new(10, 0.75));
	/// ```
	pub fn apply_decision(&self, decision: &IntersectionDecision, state: &mut PathBoundBodyState, paths: &GenericDataset<Path>) {
		assert!(decision.exit < self.path_points.len(), "Decision exit index must be < the length of points for this intersection");
		let (path_query, path_pos) = self.path_points[decision.exit].clone();
		state.path_query = paths.get_item_tuple(&path_query).expect(&format!("Intersection path query \"{:?}\" does not work", &path_query)).0.to_query();
		state.pos = path_pos;
	}
	/// Compiles Vec of all path points on given path
	/// ```
	/// use virtual_bike::prelude::*;
	/// let paths = GenericDataset::<Path> {items: vec![
	/// 	(GenericRef::id(0), Path::default()),
	/// 	(GenericRef::id(1), Path::default())
	/// ]};
	/// let intersection = Intersection::new(vec![
	/// 	(GenericQuery::id(0), PathPosition::new(1, 0.0)),
	/// 	(GenericQuery::id(1), PathPosition::new(2, 0.0)),
	/// 	(GenericQuery::id(0), PathPosition::new(3, 0.0)),
	/// 	(GenericQuery::id(1), PathPosition::new(4, 0.0)),
	/// ]);
	/// assert_eq!(
	/// 	intersection.path_points_on_specific_path(&GenericQuery::id(1), &paths),
	/// 	vec![
	/// 		PathPosition::new(2, 0.0),
	/// 		PathPosition::new(4, 0.0)
	/// 	]
	/// );
	/// ```
	pub fn path_points_on_specific_path(&self, path_query: &GenericQuery<Path>, paths: &GenericDataset<Path>) -> Vec<PathPosition> {
		let path_id = match paths.get_item_id(path_query) {
			Some(id) => id,
			None => return Vec::new()
		};
		let mut out = Vec::<PathPosition>::new();
		for (curr_path_query, path_pos) in &self.path_points {
			if let Some((curr_ref, path)) = paths.get_item_tuple(curr_path_query) {
				if curr_ref.id == path_id {
					out.push(path_pos.clone());
				}
			}
		}
		// Done
		out
	}
}

#[derive(Serialize, Deserialize, Clone)]
pub struct GenericPathSet {
	pub query_grid_scale: UInt,// 0 means no query grid
	pub intersections: GenericDataset<Intersection>,
	pub routes: GenericDataset<Route>
}

impl Default for GenericPathSet {
	fn default() -> Self {
		Self {
			query_grid_scale: 0,
			intersections: GenericDataset::new(),
			routes: GenericDataset::new()
		}
	}
}

#[derive(Serialize, Deserialize, Clone)]
pub struct SavePathSet {
	pub generic: GenericPathSet,
	pub paths: GenericDataset<SavePath>
}

#[derive(Serialize, Deserialize, Clone)]
pub struct PathSet {
	pub generic: GenericPathSet,
	pub query_grid: HashMap<IntV2, Vec<u64>>,
	pub paths: GenericDataset<Path>
}

impl PathSet {
	/// Creates a new instance of `Self` from a `SavePathSet`
	/// Because a `Path` requires an `Arc<PathType>` and a `SavePath` only has a reference to that path type (the name of a file in resources/), this function may fail if that path type config file can't be loaded.
	pub fn from_save(save: SavePathSet) -> Result<Self, String> {
		// Load path types
		let mut path_types = HashMap::<PathTypeRef, Arc<PathType>>::new();
		let mut paths = GenericDataset::<Path>::new();
		for (save_path_ref, save_path) in save.paths.items {
			// get path type config
			let path_type: Arc<PathType> = if let Some(type_arc) = path_types.get(&save_path.type_) {
				Ok(type_arc.clone())
			}
			else {
				to_string_err(resource_interface::load_path_type(&save_path.type_)).map(|og_path_type| Arc::new(og_path_type))
			}?;
			paths.items.push((save_path_ref.into_another_type::<Path>(), Path::from_save(save_path.clone(), path_type)));
		}
		// Done
		Ok(Self {
			generic: save.generic,
			query_grid: HashMap::new(),
			paths
		})
	}
	/// Gets next intersection on given path in given direction if there is one
	/// 
	/// # Returns
	/// 
	/// Option<(
	/// 	Intersection ID (u64),
	/// 	&Intersection,
	/// 	Distance along given path to intersection in direction given by `forward` (should always be 0 or positive)
	/// )>
	pub fn next_intersection_on_path(&self, path_query: &GenericQuery<Path>, path_pos: &PathPosition, forward: bool) -> Option<(u64, &Intersection, Float)> {
		// Get path
		let path: &Path = &self.paths.get_item_tuple(&path_query).expect("Invalid path reference").1;
		let path_len = path.generic.estimate_distance_to_position(&path.generic.end_position(), BCURVE_LENGTH_ESTIMATION_SEGMENTS);
		// Get distance along path
		let dist_along_path = path.generic.estimate_distance_to_position(path_pos, BCURVE_LENGTH_ESTIMATION_SEGMENTS);
		// Get all intersection points on given path
		let mut intersection_points: Vec<(u64, PathPosition)> = Vec::new();// (Intersection ID, intersection pos - `path_pos`)
		for (ref_, intersection) in &self.generic.intersections.items {
			let points: Vec<PathPosition> = intersection.path_points_on_specific_path(&path_query, &self.paths);
			for point in points {
				intersection_points.push((ref_.id, point));
			}
		}
		// Sort intersection points (unecessary)
		//intersection_points.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
		// Get closest one in correct direction (use `forward`)
		let mut out: Option<(u64, &Intersection, Float)> = None;// (IntersectionId, &Intersection, Distance from `path_pos` to this intersection point)
		for (id, curr_pos) in intersection_points {
			let curr_dist = path.generic.estimate_distance_to_position(&curr_pos, BCURVE_LENGTH_ESTIMATION_SEGMENTS);
			let diff_raw = curr_dist - dist_along_path;// From given `path_pos` to curr pos
			let diff_sign_corrected = diff_raw * bool_sign(forward) as Float;
			// Possible distance, could be `None` if path is not a loop
			let diff_positive_opt: Option<Float> = if diff_sign_corrected >= 0.0 {
				Some(diff_sign_corrected)
			}
			else {
				if path.generic.loop_ {
					let out = diff_sign_corrected + path_len;
					assert!(out >= 0.0, "The positive difference calculated between two points along path must be positive");
					assert!(out < path_len, "The distance along the path must be less then the path length. There is a possibility this is caused by a length estimation error");
					Some(out)
				}
				else {
					None
				}
			};
			// Logic to get possible new closest intersection
			if let Some(diff_positive) = diff_positive_opt {
				let mut use_this_one = match &out {
					Some((_, _, curr_best_dist)) => diff_positive < *curr_best_dist,
					None => true
				};
				// Update `out` if `new_out` is Some(_)
				if use_this_one {
					out = Some((id, &(self.generic.intersections.get_item_tuple(&GenericQuery::id(id)).expect("This shouldn't happen").1), diff_positive));
				}
			}
		}
		// Done
		out
	}
	/// Creates save path set
	/// ```
	/// use virtual_bike::prelude::*;
	/// let mut paths = PathSet::default();
	/// // TODO
	/// ```
	pub fn save(&self) -> SavePathSet {
		// Create `SavePath`s
		let mut paths = GenericDataset::<SavePath>::new();
		for (path_ref, path) in &self.paths.items {
			paths.items.push((path_ref.into_another_type::<SavePath>(), path.save()));
		}
		// Done
		SavePathSet {
			generic: self.generic.clone(),
			paths
		}
	}
}

impl Default for PathSet {
	fn default() -> Self {
		Self {
			generic: GenericPathSet::default(),
			query_grid: HashMap::new(),
			paths: GenericDataset::<Path>::new()
		}
	}
}

/// Represents a position along a path
/// ```
/// use std::cmp::Ordering;
/// use virtual_bike::prelude::PathPosition;
/// assert!(PathPosition::new(1, 0.5) > PathPosition::new(0, 0.5));
/// assert!(PathPosition::new(0, 0.5) < PathPosition::new(1, 0.5));
/// assert_eq!(PathPosition::new(0, 1.0).partial_cmp(&PathPosition::new(1, 0.0)), Some(Ordering::Equal));
/// ```
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PathPosition {
	/// The index of the first knot point encountered if moving backwards along path (or at of `t` = 0.0)
	pub latest_point: usize,
	/// The bezier curve LERP perameter, should be >=0 and < 1. Note that this does not map linearly along the length of a bezier curve.
	pub t: Float// 0<= this <= 1
}

impl PathPosition {
	pub fn new(i: usize, f: Float) -> Self {
		Self {
			latest_point: i,
			t: f
		}
	}
	fn floatify(&self) -> Float {
		(self.latest_point as Float) + self.t
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

impl PartialEq for PathPosition {
	fn eq(&self, other: &Self) -> bool {
		self.floatify() == other.floatify()
	}
	fn ne(&self, other: &Self) -> bool {
		!(self == other)
	}
}

impl PartialOrd for PathPosition {
	fn ge(&self, other: &Self) -> bool {
		self > other || self == other
	}
	fn gt(&self, other: &Self) -> bool {
		self.floatify() > other.floatify()
	}
	fn le(&self, other: &Self) -> bool {
		self < other || self == other
	}
	fn lt(&self, other: &Self) -> bool {
		self.floatify() < other.floatify()
	}
	fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
		Some(if self == other {
			Ordering::Equal
		}
		else {
			if self > other {
				Ordering::Greater
			}
			else {
				Ordering::Less
			}
		})
	}
}

/// Represents the state of a "bath-bound" vehicle
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct PathBoundBodyState {
	/// Reference to a path in the context of a `PathSet`
	pub path_query: GenericQuery<Path>,
	/// Position of body in path
	pub pos: PathPosition,
	/// in world-units / second. Wrt body
	pub velocity: Float,
	/// Whether vehicle is facing forward on path
	pub forward: bool,
	/// The vehicle may have a route, which is used for making `IntersectionDecision`s
	pub route_query_opt: Option<GenericQuery<Route>>
}

impl PathBoundBodyState {
	/// Updates self. If it goes through an intersection, it will use the optional state's route to make an `IntersectionDecision`.
	/// This will work in a loop until it has travelled the correct distance.
	pub fn update(&mut self, dt: Float, forces: &BodyForces, v_static: &VehicleStatic, path_set: &PathSet) {
		// Intersection decision loop, this will usually only run once or twice, but it in theory should handle arbitrarily large steps
		let mut step_remaining: Float = 0.0;
		loop {
			let next_intersection_opt: Option<(u64, &Intersection, Float)> = path_set.next_intersection_on_path(&self.path_query, &self.pos, self.forward);
			//let next_intersection_opt: Option<&Intersection> = next_intersection_id_opt.map(|id| paths.intersections.get(&id).expect("Invalid intersection ID"));
			let path: &Path = path_set.paths.get_item_tuple(&self.path_query).expect("Invalid path query").1;
			let step_remaining_opt: Option<f32> = path.generic.update_body(dt, &forces, v_static, self, next_intersection_opt.map(|t| t.2));
			step_remaining = match next_intersection_opt {
				Some((next_intersection_id, next_intersection, _)) => match step_remaining_opt {
					Some(step_remaining) => {// There is still distance remaining after the intersection
						// Intersection decision
						let decision: IntersectionDecision = match match &self.route_query_opt {
							Some(route_query) => {
								let route: &Route = path_set.generic.routes.get_item_tuple(&route_query).expect("This PathBoundBodyState has an invalid route ID").1;
								route.unreliable_decision_opt(GenericQuery::id(next_intersection_id), self.forward, &path_set.generic.intersections)
							},
							None => None
						} {
							Some(decision) => decision,
							None => next_intersection.default_decision(path_set, &self.path_query)
						};
						// Act on decision
						next_intersection.apply_decision(&decision, self, &path_set.paths);
						// Done
						step_remaining
					},
					None => 0.0
				},
				None => 0.0
			};
			if step_remaining == 0.0 {
				break;
			}
		}
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