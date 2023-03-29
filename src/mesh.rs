#[derive(Debug)]
pub struct BoxMesh {
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
    pub min_z: f32,
    pub max_z: f32,
}

impl BoxMesh {
    pub fn new(min_x: f32, max_x: f32, min_y: f32, max_y: f32, min_z: f32, max_z: f32) -> Self {
        Self {
            min_x,
            max_x,
            min_y,
            max_y,
            min_z,
            max_z,
        }
    }
}

#[derive(Debug)]
pub enum Mesh {
    Box(BoxMesh),
    // Cylinder,
}
