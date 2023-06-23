use rp2040_hal::{prelude::*, pwm::{InputHighRunning, Slices}};
pub trait Motor {
    fn set_power(&mut self, power:u16);
}

pub struct BrushedMotor {
    pwm: 
}

impl Motor for BrushedMotor {
    #[inline]
    fn set_power(&mut self, power: u16) {
        
    }
}
