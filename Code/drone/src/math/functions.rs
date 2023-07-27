use libm;
pub const PI: f64 = 3.141592653589793238462643383279506939937;
pub const RAD2DEGF: f32 = 180.0 / PI as f32;
pub fn cartesian_to_polar(cart: [f32; 3]) -> [f32; 3] {
    let theta = cartesian_to_polar_theta(cart);
    [cartesian_to_polar_magnitude(cart), theta[0], theta[1]]
}

#[inline(always)]
pub fn cartesian_to_polar_magnitude(cart: [f32; 3]) -> f32 {
    libm::sqrtf(cart[0] * cart[0] + cart[1] * cart[1] + cart[2] * cart[2])
}

#[inline(always)]
pub fn cartesian_to_polar_theta(cart: [f32; 3]) -> [f32; 2] {
    let tx = libm::atan2f(cart[0], cart[2]) * RAD2DEGF;
    let ty = libm::atan2f(cart[1], cart[2]) * RAD2DEGF;
    [tx, ty]
}
