use serde::{Serialize, Deserialize};


#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Vector3D {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}
