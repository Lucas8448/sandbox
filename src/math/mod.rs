use std::ops::{Add, Mul, Neg, Sub};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn magnitude(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn magnitude_sq(self) -> f64 {
        self.dot(self)
    }

    pub fn normalize(self) -> Self {
        let m = self.magnitude();
        if m < 1e-12 {
            Self::zero()
        } else {
            self * (1.0 / m)
        }
    }

    pub fn lerp(self, other: Self, t: f64) -> Self {
        self * (1.0 - t) + other * t
    }

    pub fn scale(self, s: f64) -> Self {
        self * s
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self {
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
        }
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn identity() -> Self {
        Self {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Self {
        let half = angle * 0.5;
        let s = half.sin();
        let a = axis.normalize();
        Self {
            w: half.cos(),
            x: a.x * s,
            y: a.y * s,
            z: a.z * s,
        }
    }

    pub fn magnitude(self) -> f64 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalize(self) -> Self {
        let m = self.magnitude();
        if m < 1e-12 {
            Self::identity()
        } else {
            let inv = 1.0 / m;
            Self {
                w: self.w * inv,
                x: self.x * inv,
                y: self.y * inv,
                z: self.z * inv,
            }
        }
    }

    pub fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    pub fn multiply(self, other: Self) -> Self {
        Self {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }

    pub fn rotate_vector(self, v: Vec3) -> Vec3 {
        let qv = Quaternion::new(0.0, v.x, v.y, v.z);
        let result = self.multiply(qv).multiply(self.conjugate());
        Vec3::new(result.x, result.y, result.z)
    }

    pub fn slerp(self, other: Self, t: f64) -> Self {
        let mut dot = self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z;
        let mut other = other;
        if dot < 0.0 {
            other = Quaternion::new(-other.w, -other.x, -other.y, -other.z);
            dot = -dot;
        }
        if dot > 0.9995 {
            // Linear interpolation for very close quaternions
            let result = Quaternion::new(
                self.w + t * (other.w - self.w),
                self.x + t * (other.x - self.x),
                self.y + t * (other.y - self.y),
                self.z + t * (other.z - self.z),
            );
            return result.normalize();
        }
        let theta = dot.acos();
        let sin_theta = theta.sin();
        let wa = ((1.0 - t) * theta).sin() / sin_theta;
        let wb = (t * theta).sin() / sin_theta;
        Quaternion::new(
            wa * self.w + wb * other.w,
            wa * self.x + wb * other.x,
            wa * self.y + wb * other.y,
            wa * self.z + wb * other.z,
        )
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Transform {
    pub position: Vec3,
    pub orientation: Quaternion,
}

impl Transform {
    pub fn new(position: Vec3, orientation: Quaternion) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn identity() -> Self {
        Self {
            position: Vec3::zero(),
            orientation: Quaternion::identity(),
        }
    }

    pub fn transform_point(&self, point: Vec3) -> Vec3 {
        self.orientation.rotate_vector(point) + self.position
    }

    pub fn transform_direction(&self, dir: Vec3) -> Vec3 {
        self.orientation.rotate_vector(dir)
    }

    pub fn inverse(&self) -> Self {
        let inv_rot = self.orientation.conjugate();
        Self {
            position: inv_rot.rotate_vector(-self.position),
            orientation: inv_rot,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vec3_operations() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        assert_eq!(a + b, Vec3::new(5.0, 7.0, 9.0));
        assert_eq!(a - b, Vec3::new(-3.0, -3.0, -3.0));
        assert_eq!(a * 2.0, Vec3::new(2.0, 4.0, 6.0));
        assert!((a.dot(b) - 32.0).abs() < 1e-10);
        assert_eq!(a.cross(b), Vec3::new(-3.0, 6.0, -3.0));
        let n = Vec3::new(3.0, 0.0, 0.0).normalize();
        assert!((n.magnitude() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn quaternion_rotation() {
        let q = Quaternion::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_2);
        let v = Vec3::new(1.0, 0.0, 0.0);
        let rotated = q.rotate_vector(v);
        assert!((rotated.x - 0.0).abs() < 1e-10);
        assert!((rotated.y - 1.0).abs() < 1e-10);
        assert!((rotated.z - 0.0).abs() < 1e-10);
    }
}
