#[derive(Debug)]
pub struct Motor {
    power: f32,
    port: u8,
}

impl Motor {
    pub fn new(port: u8) -> Self {
        Self { power: 0.0, port }
    }
    pub fn set_power(&mut self, power: f32) {
        self.power = power;
    }
}
