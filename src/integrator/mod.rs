use crate::dynamics::RigidBody;
use crate::environment::Environment;
use crate::math::{Quaternion, Vec3};

pub trait Integrator {
    fn step(&self, body: &mut RigidBody, dt: f64, env: &Environment);
}

/// Simple forward Euler integration
pub struct EulerIntegrator;

impl Integrator for EulerIntegrator {
    fn step(&self, body: &mut RigidBody, dt: f64, env: &Environment) {
        // Apply environment forces
        let gravity = env.gravity_force(body.mass);
        body.apply_force(gravity, None);

        if body.drag_coefficient > 0.0 {
            let drag = env.drag_force(
                body.velocity,
                body.drag_coefficient,
                body.cross_section_area,
            );
            body.apply_force(drag, None);
        }

        // Linear integration
        let accel = body.linear_acceleration();
        body.velocity = body.velocity + accel * dt;
        body.position = body.position + body.velocity * dt;

        // Angular integration
        let alpha = body.angular_acceleration();
        body.angular_velocity = body.angular_velocity + alpha * dt;

        // Quaternion integration
        let w = body.angular_velocity;
        let omega_quat = Quaternion::new(0.0, w.x, w.y, w.z);
        let dq = omega_quat.multiply(body.orientation);
        body.orientation = Quaternion::new(
            body.orientation.w + 0.5 * dq.w * dt,
            body.orientation.x + 0.5 * dq.x * dt,
            body.orientation.y + 0.5 * dq.y * dt,
            body.orientation.z + 0.5 * dq.z * dt,
        )
        .normalize();

        body.clear_forces();
    }
}

pub struct RK4Integrator;

#[derive(Clone)]
struct State {
    position: Vec3,
    velocity: Vec3,
    orientation: Quaternion,
    angular_velocity: Vec3,
}

struct Derivative {
    dx: Vec3, // velocity
    dv: Vec3, // acceleration
    dq: Quaternion,
    dw: Vec3, // angular acceleration
}

impl RK4Integrator {
    fn compute_derivative(body: &RigidBody, state: &State, env: &Environment) -> Derivative {
        // Recompute forces based on state
        let gravity = env.gravity_force(body.mass);
        let mut force = body.force + gravity;

        if body.drag_coefficient > 0.0 {
            let drag = env.drag_force(
                state.velocity,
                body.drag_coefficient,
                body.cross_section_area,
            );
            force = force + drag;
        }

        let accel = force * (1.0 / body.mass);
        let alpha = Vec3::new(
            body.torque.x / body.inertia.x,
            body.torque.y / body.inertia.y,
            body.torque.z / body.inertia.z,
        );

        let w = state.angular_velocity;
        let omega_quat = Quaternion::new(0.0, w.x, w.y, w.z);
        let dq_raw = omega_quat.multiply(state.orientation);
        let dq = Quaternion::new(
            0.5 * dq_raw.w,
            0.5 * dq_raw.x,
            0.5 * dq_raw.y,
            0.5 * dq_raw.z,
        );

        Derivative {
            dx: state.velocity,
            dv: accel,
            dq,
            dw: alpha,
        }
    }
}

impl Integrator for RK4Integrator {
    fn step(&self, body: &mut RigidBody, dt: f64, env: &Environment) {
        let initial = State {
            position: body.position,
            velocity: body.velocity,
            orientation: body.orientation,
            angular_velocity: body.angular_velocity,
        };

        // k1
        let k1 = Self::compute_derivative(body, &initial, env);

        // k2
        let s2 = State {
            position: initial.position + k1.dx * (dt * 0.5),
            velocity: initial.velocity + k1.dv * (dt * 0.5),
            orientation: Quaternion::new(
                initial.orientation.w + k1.dq.w * (dt * 0.5),
                initial.orientation.x + k1.dq.x * (dt * 0.5),
                initial.orientation.y + k1.dq.y * (dt * 0.5),
                initial.orientation.z + k1.dq.z * (dt * 0.5),
            )
            .normalize(),
            angular_velocity: initial.angular_velocity + k1.dw * (dt * 0.5),
        };
        let k2 = Self::compute_derivative(body, &s2, env);

        // k3
        let s3 = State {
            position: initial.position + k2.dx * (dt * 0.5),
            velocity: initial.velocity + k2.dv * (dt * 0.5),
            orientation: Quaternion::new(
                initial.orientation.w + k2.dq.w * (dt * 0.5),
                initial.orientation.x + k2.dq.x * (dt * 0.5),
                initial.orientation.y + k2.dq.y * (dt * 0.5),
                initial.orientation.z + k2.dq.z * (dt * 0.5),
            )
            .normalize(),
            angular_velocity: initial.angular_velocity + k2.dw * (dt * 0.5),
        };
        let k3 = Self::compute_derivative(body, &s3, env);

        // k4
        let s4 = State {
            position: initial.position + k3.dx * dt,
            velocity: initial.velocity + k3.dv * dt,
            orientation: Quaternion::new(
                initial.orientation.w + k3.dq.w * dt,
                initial.orientation.x + k3.dq.x * dt,
                initial.orientation.y + k3.dq.y * dt,
                initial.orientation.z + k3.dq.z * dt,
            )
            .normalize(),
            angular_velocity: initial.angular_velocity + k3.dw * dt,
        };
        let k4 = Self::compute_derivative(body, &s4, env);

        // Combine
        let sixth = 1.0 / 6.0;
        body.position =
            initial.position + (k1.dx + k2.dx * 2.0 + k3.dx * 2.0 + k4.dx) * (dt * sixth);
        body.velocity =
            initial.velocity + (k1.dv + k2.dv * 2.0 + k3.dv * 2.0 + k4.dv) * (dt * sixth);

        body.orientation = Quaternion::new(
            initial.orientation.w
                + (k1.dq.w + k2.dq.w * 2.0 + k3.dq.w * 2.0 + k4.dq.w) * (dt * sixth),
            initial.orientation.x
                + (k1.dq.x + k2.dq.x * 2.0 + k3.dq.x * 2.0 + k4.dq.x) * (dt * sixth),
            initial.orientation.y
                + (k1.dq.y + k2.dq.y * 2.0 + k3.dq.y * 2.0 + k4.dq.y) * (dt * sixth),
            initial.orientation.z
                + (k1.dq.z + k2.dq.z * 2.0 + k3.dq.z * 2.0 + k4.dq.z) * (dt * sixth),
        )
        .normalize();

        body.angular_velocity =
            initial.angular_velocity + (k1.dw + k2.dw * 2.0 + k3.dw * 2.0 + k4.dw) * (dt * sixth);

        body.clear_forces();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rk4_freefall_accuracy() {
        // Free-fall for 2 seconds, should match y = -0.5*g*t^2
        let env = Environment::earth();
        let integrator = RK4Integrator;
        let mut body = RigidBody::new(1.0);
        let dt = 0.001;
        let steps = 2000; // 2 seconds

        for _ in 0..steps {
            integrator.step(&mut body, dt, &env);
        }

        let t = 2.0;
        let expected_y = -0.5 * 9.81 * t * t;
        let error = ((body.position.y - expected_y) / expected_y).abs();
        assert!(
            error < 0.001,
            "RK4 free-fall error: {:.6}%, pos.y={}, expected={}",
            error * 100.0,
            body.position.y,
            expected_y
        );
    }
}
