use crate::math::Vec3;

#[derive(Debug, Clone)]
pub struct Environment {
    pub gravity: Vec3,
    pub air_density: f64,
    pub wind: Vec3,
}

impl Environment {
    pub fn new(gravity: Vec3, air_density: f64, wind: Vec3) -> Self {
        Self {
            gravity,
            air_density,
            wind,
        }
    }

    /// Earth-standard: g = -9.81 m/s² (Y-up), sea-level density, no wind
    pub fn earth() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            air_density: 1.225,
            wind: Vec3::zero(),
        }
    }

    /// No gravity, no atmosphere
    pub fn space() -> Self {
        Self {
            gravity: Vec3::zero(),
            air_density: 0.0,
            wind: Vec3::zero(),
        }
    }

    /// Compute aerodynamic drag force on a body
    pub fn drag_force(
        &self,
        velocity: Vec3,
        drag_coefficient: f64,
        cross_section_area: f64,
    ) -> Vec3 {
        let relative_vel = velocity - self.wind;
        let speed = relative_vel.magnitude();
        if speed < 1e-12 {
            return Vec3::zero();
        }
        // F_drag = -0.5 * rho * Cd * A * |v|^2 * v_hat
        let magnitude =
            0.5 * self.air_density * drag_coefficient * cross_section_area * speed * speed;
        relative_vel.normalize() * (-magnitude)
    }

    /// Simple atmospheric density model
    pub fn density_at_altitude(&self, altitude: f64) -> f64 {
        // Scale height ~8500m for Earth
        self.air_density * (-altitude / 8500.0).exp()
    }

    pub fn gravity_force(&self, mass: f64) -> Vec3 {
        self.gravity * mass
    }
}

impl Default for Environment {
    fn default() -> Self {
        Self::earth()
    }
}
