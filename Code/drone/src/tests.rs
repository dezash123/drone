use crate::math::functions::atan;
use defmt::info;
pub fn test() {
    info!("{}", (atan(1.0, 100)));
}
