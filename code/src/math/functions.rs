use libm;
pub const PI: f64 = 3.141592653589793238462643383279506939937;
pub const RAD2DEGF: f32 = 180.0 / PI as f32;
pub fn cartesian_to_polar<const N: usize>(cart: [f32; N], degrees: bool) -> [f32; N]
where
    [(); N - 1]:,
{
    let mut polar = [0.0f32; N];
    polar[0] = cartesian_to_polar_magnitude(cart);
    polar.copy_from_slice(&cartesian_to_polar_theta(cart, degrees));
    polar
}

#[inline(always)]
pub fn cartesian_to_polar_magnitude<const N: usize>(cart: [f32; N]) -> f32 {
    let mut mag: f32 = 0.0;
    for i in 0..N {
        mag += cart[i] * cart[i];
    }
    libm::sqrtf(mag)
}
// ik this is a weird implementation but it makes stuff make sense
#[inline(always)]
pub fn cartesian_to_polar_theta<const N: usize>(cart: [f32; N], degrees: bool) -> [f32; N - 1] {
    let mut theta = [0.0; N - 1];
    for i in 0..N - 1 {
        theta[N - 2 - i] = -libm::atan2f(cart[i], cart[N - 1]);
        if degrees {
            theta[N - 2 - i] *= RAD2DEGF;
        }
    }
    theta
}

pub struct PD {
    pub kP: f32,
    pub kD: f32,
    last_error: f32,
}

impl PD {
    pub fn get_next(&mut self, error: f32, dt: f32) -> f32 {
        
    }
}

pub struct PID {
    pub pd: PD,
    pub kI: f32,
    integral: f32,
}

impl PID {
    pub fn get_next(&mut self, error: f32, dt: f32) -> f32 {
        self.integral += 
    }
}
