use crate::math::{Quaternion, Vec3};

#[derive(Debug, Clone)]
pub struct RigidBody {
    pub mass: f64,
    pub inertia: Vec3, // diagonal inertia tensor (Ixx, Iyy, Izz)
    pub position: Vec3,
    pub velocity: Vec3,
    pub orientation: Quaternion,
    pub angular_velocity: Vec3,
    pub force: Vec3,
    pub torque: Vec3,
    pub drag_coefficient: f64,
    pub cross_section_area: f64,
}

impl RigidBody {
    pub fn new(mass: f64) -> Self {
        Self {
            mass,
            inertia: Vec3::new(1.0, 1.0, 1.0),
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            orientation: Quaternion::identity(),
            angular_velocity: Vec3::zero(),
            force: Vec3::zero(),
            torque: Vec3::zero(),
            drag_coefficient: 0.0,
            cross_section_area: 0.0,
        }
    }

    pub fn with_position(mut self, pos: Vec3) -> Self {
        self.position = pos;
        self
    }

    pub fn with_velocity(mut self, vel: Vec3) -> Self {
        self.velocity = vel;
        self
    }

    pub fn with_inertia(mut self, inertia: Vec3) -> Self {
        self.inertia = inertia;
        self
    }

    pub fn with_drag(mut self, coefficient: f64, area: f64) -> Self {
        self.drag_coefficient = coefficient;
        self.cross_section_area = area;
        self
    }

    pub fn with_orientation(mut self, q: Quaternion) -> Self {
        self.orientation = q;
        self
    }

    pub fn apply_force(&mut self, force: Vec3, point: Option<Vec3>) {
        self.force = self.force + force;
        if let Some(p) = point {
            let r = p - self.position;
            self.torque = self.torque + r.cross(force);
        }
    }

    pub fn apply_torque(&mut self, torque: Vec3) {
        self.torque = self.torque + torque;
    }

    pub fn clear_forces(&mut self) {
        self.force = Vec3::zero();
        self.torque = Vec3::zero();
    }

    pub fn linear_acceleration(&self) -> Vec3 {
        self.force * (1.0 / self.mass)
    }

    /// Angular acceleration in the same frame as `inertia` and
    /// `angular_velocity` (the principal-axis / world-aligned frame).
    /// Includes the gyroscopic / Euler term `ω × (I·ω)`, so a torque-free
    /// body with non-isotropic inertia precesses correctly instead of
    /// drifting linearly.
    ///
    /// I·ω̇ = τ − ω × (I·ω)
    pub fn angular_acceleration(&self) -> Vec3 {
        angular_acceleration(self.torque, self.angular_velocity, self.inertia)
    }

    pub fn kinetic_energy(&self) -> f64 {
        let linear = 0.5 * self.mass * self.velocity.magnitude_sq();
        let w = self.angular_velocity;
        let rotational = 0.5
            * (self.inertia.x * w.x * w.x
                + self.inertia.y * w.y * w.y
                + self.inertia.z * w.z * w.z);
        linear + rotational
    }

    pub fn momentum(&self) -> Vec3 {
        self.velocity * self.mass
    }

    pub fn speed(&self) -> f64 {
        self.velocity.magnitude()
    }
}

/// Standalone form of `RigidBody::angular_acceleration` so integrators can
/// evaluate the rotational ODE at intermediate states without mutating the body.
pub fn angular_acceleration(torque: Vec3, angular_velocity: Vec3, inertia: Vec3) -> Vec3 {
    let iw = Vec3::new(
        inertia.x * angular_velocity.x,
        inertia.y * angular_velocity.y,
        inertia.z * angular_velocity.z,
    );
    let gyro = angular_velocity.cross(iw);
    Vec3::new(
        (torque.x - gyro.x) / inertia.x,
        (torque.y - gyro.y) / inertia.y,
        (torque.z - gyro.z) / inertia.z,
    )
}
