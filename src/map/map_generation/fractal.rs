// Fractal terrain generation
use serde::{Serialize, Deserialize};
use crate::prelude::*;
use super::Random;

#[derive(Serialize, Deserialize, Clone)]
pub struct FractalCreator {

}

impl FractalCreator {
    pub fn create_mesh(&self, args: MeshCreationArgs) -> RegularElevationMesh {
        MapGenerator::RandomGen(Random::flat()).create_mesh(args)// TODO
    }
}