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

#[inline(always)]
pub fn dot_product<const N: usize>(v1: [f32; N], v2: [f32; N]) -> f32 {
    let mut product = v1[0] * v2[0];
    for i in 1..N {
        if v1[i] != 0.0 && v2[i] != 0.0 {
            product += v1[i] * v2[i];
        }
    }
    product
}

pub fn matrix_multiply<const N: usize, const M: usize, const K: usize>(
    m1: [[f32; M]; N],
    m2: [[f32; K]; M],
) -> [[f32; K]; N] {
    let b = transpose(m2);
    let mut product = [[0.0; K]; N];
    for n in 0..N {
        for k in 0..K {
            product[n][k] = dot_product(m1[n], b[k]);
        }
    }
    product
}

pub fn transpose<const N: usize, const K: usize>(m: [[f32; K]; N]) -> [[f32; N]; K] {
    let mut transposed = [[0.0; N]; K];
    for n in 0..N {
        for k in 0..K {
            transposed[k][n] = m[n][k];
        }
    }
    transposed
}

pub struct PD {
    pub k_p: f32,
    pub k_d: f32,
    last_error: f32,
    last_v: f32,
}

impl PD {
    pub fn new(k_p: f32, k_d: f32) -> Self {
        Self {
            k_p,
            k_d,
            last_error: 0.0,
            last_v: 0.0,
        }
    }
    pub fn get_next(&mut self, error: f32, dt: f32) -> f32 {
        let v = if dt == 0.0 {
            self.last_v
        } else {
            (error - self.last_error) / dt
        };
        self.last_v = v;
        self.last_error = error;
        self.k_p * error + self.k_d * v
    }
}

pub struct PID {
    pub pd: PD,
    pub k_i: f32,
    integral: f32,
}

impl PID {
    pub fn new(k_p: f32, k_i: f32, k_d: f32) -> Self {
        Self {
            pd: PD::new(k_p, k_d),
            k_i,
            integral: 0.0,
        }
    }
    pub fn get_next(&mut self, error: f32, dt: f32) -> f32 {
        self.integral += self.k_i * dt * error;
        self.pd.get_next(error, dt) + self.integral
    }
}

pub struct KalmanFilter {
    pub x: f32,
    bias: f32,
    q_x: f32,
    q_bias: f32,
    r_measure: f32,
    p: [[f32; 2]; 2],
}

impl KalmanFilter {
    pub fn new(q_x: f32, q_bias: f32, r_measure: f32) -> Self {
        Self {
            x: 0.0,
            bias: 0.0,
            q_x,
            q_bias,
            r_measure,
            p: [[0.0; 2]; 2],
        }
    }
    pub fn push(&mut self, new_x: f32, new_v: f32, dt: f32) {
        let v = new_v - self.bias;
        self.x += v * dt;

        self.p[0][0] += dt * (dt * self.p[1][1] - self.p[0][1] - self.p[1][0] + self.q_x);
        self.p[0][1] -= dt * self.p[1][1];
        self.p[1][0] -= dt * self.p[1][1];
        self.p[1][1] += self.q_bias * dt;

        let s = self.p[0][0] + self.r_measure;

        let mut k = [0.0; 2];
        k[0] = self.p[0][0] / s;
        k[1] = self.p[1][0] / s;

        let y = new_x - self.x;
        self.x += k[0] * y;
        self.bias += k[1] * y;

        let p00_temp = self.p[0][0];
        let p01_temp = self.p[0][1];

        self.p[0][0] -= k[0] * p00_temp;
        self.p[0][1] -= k[0] * p01_temp;
        self.p[1][0] -= k[1] * p00_temp;
        self.p[1][1] -= k[1] * p01_temp;
    }
}

pub struct ComplementaryFilter2 {
    last_value: f32,
    weights: [f32; 2],
}

impl ComplementaryFilter {
    pub fn new(weights: [f32; 2]) -> Self {
        Self {
            0.0,
            0.0,
            weights,
        }
    }
    pub fn get(x: f32, v: f32, dt: f32) -> f32 {
        self.last_value = weights[0] * x + weights[1] * (last_value + v * dt);
        self.last_value
    }
}
        
