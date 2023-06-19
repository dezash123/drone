use crate::control::motor::Motor;
const FR_PORT: u8 = 1;
const FL_PORT: u8 = 2;
const BR_PORT: u8 = 3;
const BL_PORT: u8 = 4;

#[derive(Debug)]
pub struct FlightSystem {
    front_left: Motor,
    back_left: Motor,
    front_right: Motor,
    back_right: Motor,
}
impl FlightSystem {
    pub fn new() -> Self {
        Self {
            front_left: Motor::new(FL_PORT),
            back_left: Motor::new(BL_PORT),
            front_right: Motor::new(FR_PORT),
            back_right: Motor::new(BR_PORT),
        }
    }
    fn calc_speeds() -> [f32; 4] {
        [0.0, 0.0, 0.0, 0.0]
    }
    pub fn update(&mut self) {
        let speeds: [f32; 4] = Self::calc_speeds();
        self.front_left.set_power(speeds[0]);
        self.front_right.set_power(speeds[1]);
        self.back_right.set_power(speeds[2]);
        self.back_left.set_power(speeds[3]);
    }
}
