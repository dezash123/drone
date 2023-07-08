use core::ops::Add;
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}
impl<T: Clone> Vector3<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
    #[inline(always)]
    pub fn from(array: &[T; 3]) -> Self {
        Self {
            x: array[0].clone(),
            y: array[1].clone(),
            z: array[2].clone(),
        }
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
