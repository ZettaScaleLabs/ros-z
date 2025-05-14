use serde::{Serialize, Deserialize};


#[derive(Debug, Serialize, Deserialize, Default, Clone, Copy)]
pub struct Vector3D {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct ByteMultiArray {
    pub layout: MultiArrayLayout,
    pub data: Vec<u8>,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct MultiArrayLayout {
    pub dim: Vec<MultiArrayDimension>,
    pub data_offset: u32,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct MultiArrayDimension {
    pub label: String,
    pub size: u32,
    pub stride: u32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use cdr::{CdrLe, Infinite};

    #[test]
    fn test_bytes() {
        let msg = ByteMultiArray::default();
        cdr::serialize::<_, _, CdrLe>(&msg, Infinite)
            .unwrap();
    }

}
