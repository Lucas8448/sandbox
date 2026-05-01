use crate::math::Vec3;

/// How air density falls off with altitude.
#[derive(Debug, Clone)]
pub enum AirDensityModel {
    /// Density is the same everywhere (matches v0.1.x behavior).
    Constant,
    /// Isothermal exponential atmosphere: ρ(h) = ρ₀ · exp(-h / scale_height).
    /// Earth-like default scale height ≈ 8500 m.
    Exponential { scale_height: f64 },
}

/// How gravity varies in space.
#[derive(Debug, Clone)]
pub enum GravityModel {
    /// Uniform field — `Environment::gravity` is applied everywhere
    /// (matches v0.1.x behavior).
    Uniform,
    /// Newtonian inverse-square: |g(h)| = surface_gravity · (R / (R + h))²,
    /// pointed in the −`up` direction. Altitude is `position · up` (so
    /// origin sits on the planet surface, not the planet center).
    InverseSquare {
        surface_gravity: f64,
        planet_radius: f64,
        up: Vec3,
    },
}

#[derive(Debug, Clone)]
pub struct Environment {
    /// Sea-level / reference gravity vector. Used directly when
    /// `gravity_model = Uniform`. Magnitude is used as the fallback
    /// surface gravity for the inverse-square model otherwise.
    pub gravity: Vec3,
    /// Sea-level / reference air density (kg/m³).
    pub air_density: f64,
    pub wind: Vec3,
    pub density_model: AirDensityModel,
    pub gravity_model: GravityModel,
}

impl Environment {
    pub fn new(gravity: Vec3, air_density: f64, wind: Vec3) -> Self {
        Self {
            gravity,
            air_density,
            wind,
            density_model: AirDensityModel::Constant,
            gravity_model: GravityModel::Uniform,
        }
    }

    /// Earth-standard: g = -9.81 m/s² (Y-up), sea-level density, no wind.
    /// Density and gravity are uniform — call `with_exponential_atmosphere`
    /// or `with_inverse_square_gravity` for higher-fidelity flight.
    pub fn earth() -> Self {
        Self::new(Vec3::new(0.0, -9.81, 0.0), 1.225, Vec3::zero())
    }

    /// No gravity, no atmosphere
    pub fn space() -> Self {
        Self::new(Vec3::zero(), 0.0, Vec3::zero())
    }

    /// Builder: enable an exponential atmosphere with the given scale height (meters).
    /// Earth's troposphere is ≈ 8500 m.
    pub fn with_exponential_atmosphere(mut self, scale_height: f64) -> Self {
        self.density_model = AirDensityModel::Exponential { scale_height };
        self
    }

    /// Builder: enable Newtonian inverse-square gravity. `up` is normalized.
    /// Earth: surface_gravity = 9.81, planet_radius = 6.371e6, up = (0,1,0).
    pub fn with_inverse_square_gravity(mut self, surface_gravity: f64, planet_radius: f64, up: Vec3) -> Self {
        let up = up.normalize();
        self.gravity_model = GravityModel::InverseSquare {
            surface_gravity,
            planet_radius,
            up,
        };
        // Keep `gravity` consistent at h=0 for users that read it directly.
        self.gravity = up * (-surface_gravity);
        self
    }

    /// Air density at the given altitude (meters above the reference).
    pub fn density_at(&self, altitude: f64) -> f64 {
        match self.density_model {
            AirDensityModel::Constant => self.air_density,
            AirDensityModel::Exponential { scale_height } => {
                if scale_height <= 0.0 {
                    self.air_density
                } else {
                    // Below the reference altitude (e.g. underground / underwater)
                    // density grows; clamp to a sane upper bound to avoid blow-up.
                    self.air_density * (-altitude / scale_height).exp()
                }
            }
        }
    }

    /// Backwards-compatible alias for `density_at`. Always uses the exponential
    /// model with Earth-like 8500 m scale height regardless of the configured
    /// `density_model`, matching v0.1.x semantics.
    pub fn density_at_altitude(&self, altitude: f64) -> f64 {
        self.air_density * (-altitude / 8500.0).exp()
    }

    /// Compute aerodynamic drag force on a body at the given world position.
    /// Uses `density_at(altitude)` so altitude-dependent atmospheres take effect.
    pub fn drag_force(
        &self,
        velocity: Vec3,
        position: Vec3,
        drag_coefficient: f64,
        cross_section_area: f64,
    ) -> Vec3 {
        let relative_vel = velocity - self.wind;
        let speed = relative_vel.magnitude();
        if speed < 1e-12 {
            return Vec3::zero();
        }
        let altitude = self.altitude_at(position);
        let rho = self.density_at(altitude);
        // F_drag = -0.5 * rho * Cd * A * |v|^2 * v_hat
        let magnitude = 0.5 * rho * drag_coefficient * cross_section_area * speed * speed;
        relative_vel.normalize() * (-magnitude)
    }

    /// Gravity force on a body of `mass` at world `position`.
    pub fn gravity_force(&self, mass: f64, position: Vec3) -> Vec3 {
        match self.gravity_model {
            GravityModel::Uniform => self.gravity * mass,
            GravityModel::InverseSquare {
                surface_gravity,
                planet_radius,
                up,
            } => {
                let altitude = position.dot(up);
                let r = planet_radius + altitude;
                if r <= 0.0 {
                    // Inside / at the planet center: fall back to surface gravity
                    // to avoid singularities.
                    return up * (-surface_gravity * mass);
                }
                let ratio = planet_radius / r;
                let g = surface_gravity * ratio * ratio;
                up * (-g * mass)
            }
        }
    }

    /// Altitude convention: for `Uniform` gravity, `position.y` is altitude.
    /// For `InverseSquare`, altitude is `position · up`.
    pub fn altitude_at(&self, position: Vec3) -> f64 {
        match self.gravity_model {
            GravityModel::Uniform => position.y,
            GravityModel::InverseSquare { up, .. } => position.dot(up),
        }
    }
}

impl Default for Environment {
    fn default() -> Self {
        Self::earth()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constant_density_unchanged_with_altitude() {
        let env = Environment::earth();
        assert_eq!(env.density_at(0.0), 1.225);
        assert_eq!(env.density_at(10_000.0), 1.225);
    }

    #[test]
    fn exponential_atmosphere_falls_off() {
        let env = Environment::earth().with_exponential_atmosphere(8500.0);
        let rho_0 = env.density_at(0.0);
        let rho_h = env.density_at(8500.0);
        // One scale height up: ρ should be ρ₀/e
        let expected = rho_0 / std::f64::consts::E;
        assert!((rho_h - expected).abs() < 1e-9, "got {}, expected {}", rho_h, expected);
    }

    #[test]
    fn drag_decreases_with_altitude_in_exponential_atmosphere() {
        let env = Environment::earth().with_exponential_atmosphere(8500.0);
        let v = Vec3::new(100.0, 0.0, 0.0);
        let f_low = env.drag_force(v, Vec3::new(0.0, 0.0, 0.0), 0.5, 1.0).magnitude();
        let f_high = env.drag_force(v, Vec3::new(0.0, 30_000.0, 0.0), 0.5, 1.0).magnitude();
        assert!(f_low > f_high * 10.0, "expected order-of-magnitude drop, got {} -> {}", f_low, f_high);
    }

    #[test]
    fn inverse_square_gravity_quarters_at_one_planet_radius_up() {
        let env = Environment::earth().with_inverse_square_gravity(9.81, 6.371e6, Vec3::new(0.0, 1.0, 0.0));
        let g_surface = env.gravity_force(1.0, Vec3::zero()).magnitude();
        let g_high = env.gravity_force(1.0, Vec3::new(0.0, 6.371e6, 0.0)).magnitude();
        assert!((g_surface - 9.81).abs() < 1e-9);
        // At altitude = R, g should be g0 / 4
        assert!((g_high - 9.81 / 4.0).abs() < 1e-6, "got {}", g_high);
    }

    #[test]
    fn uniform_gravity_unchanged_with_altitude() {
        let env = Environment::earth();
        let g_low = env.gravity_force(1.0, Vec3::zero()).magnitude();
        let g_high = env.gravity_force(1.0, Vec3::new(0.0, 1e6, 0.0)).magnitude();
        assert!((g_low - g_high).abs() < 1e-12);
    }
}
