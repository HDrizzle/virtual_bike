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