use core::ops::Add;
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector3<T> {
    x: T,
    y: T,
    z: T,
}
impl<T> Vector3<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}
impl<T: Add<Output = T>> Add for Vector3<T> {
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}
