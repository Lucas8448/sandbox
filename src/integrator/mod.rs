use crate::dynamics::{angular_acceleration, RigidBody};
use crate::environment::Environment;
use crate::math::{Quaternion, Vec3};

pub trait Integrator {
    fn step(&self, body: &mut RigidBody, dt: f64, env: &Environment);
}

/// Semi-implicit (symplectic) Euler. Cheap and energy-stable for oscillators,
/// but only first-order accurate. Prefer `RK4Integrator` when accuracy matters.
pub struct EulerIntegrator;

impl Integrator for EulerIntegrator {
    fn step(&self, body: &mut RigidBody, dt: f64, env: &Environment) {
        // Apply environment forces (evaluated at current state)
        let gravity = env.gravity_force(body.mass, body.position);
        body.apply_force(gravity, None);

        if body.drag_coefficient > 0.0 {
            let drag = env.drag_force(
                body.velocity,
                body.position,
                body.drag_coefficient,
                body.cross_section_area,
            );
            body.apply_force(drag, None);
        }

        // Linear (semi-implicit: update velocity first, then position with new v)
        let accel = body.linear_acceleration();
        body.velocity = body.velocity + accel * dt;
        body.position = body.position + body.velocity * dt;

        // Angular (uses gyroscopic-aware ω̇)
        let alpha = body.angular_acceleration();
        body.angular_velocity = body.angular_velocity + alpha * dt;

        // Quaternion integration:  q̇ = 0.5 · ω_quat ⊗ q
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

/// Classical 4th-order Runge–Kutta. Re-evaluates state-dependent forces
/// (gravity, drag, gyroscopic torque) at each of the four sub-stages.
pub struct RK4Integrator;

#[derive(Clone, Copy)]
struct State {
    position: Vec3,
    velocity: Vec3,
    orientation: Quaternion,
    angular_velocity: Vec3,
}

#[derive(Clone, Copy)]
struct Derivative {
    dx: Vec3,        // velocity
    dv: Vec3,        // acceleration
    dq: Quaternion,  // q̇ (already includes the 0.5 factor)
    dw: Vec3,        // angular acceleration (incl. gyroscopic term)
}

impl RK4Integrator {
    fn compute_derivative(body: &RigidBody, state: &State, env: &Environment) -> Derivative {
        // External / applied force is held constant across the step (the user's
        // `apply_force` call is treated as a constant load over [t, t+dt]).
        // State-dependent forces (gravity in non-uniform fields, drag) are
        // re-evaluated at the sub-stage state.
        let gravity = env.gravity_force(body.mass, state.position);
        let mut force = body.force + gravity;

        if body.drag_coefficient > 0.0 {
            let drag = env.drag_force(
                state.velocity,
                state.position,
                body.drag_coefficient,
                body.cross_section_area,
            );
            force = force + drag;
        }

        let accel = force * (1.0 / body.mass);

        // Gyroscopic-aware angular acceleration evaluated at the sub-stage ω.
        let alpha = angular_acceleration(body.torque, state.angular_velocity, body.inertia);

        // q̇ = 0.5 · ω ⊗ q   (world-frame angular velocity convention)
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

    fn advance(initial: &State, d: &Derivative, h: f64) -> State {
        // Note: we deliberately do NOT normalize the intermediate quaternion.
        // Normalizing each sub-stage biases the RK4 weighted sum away from the
        // true derivative; we normalize once at the end of the full step.
        State {
            position: initial.position + d.dx * h,
            velocity: initial.velocity + d.dv * h,
            orientation: Quaternion::new(
                initial.orientation.w + d.dq.w * h,
                initial.orientation.x + d.dq.x * h,
                initial.orientation.y + d.dq.y * h,
                initial.orientation.z + d.dq.z * h,
            ),
            angular_velocity: initial.angular_velocity + d.dw * h,
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

        let k1 = Self::compute_derivative(body, &initial, env);
        let s2 = Self::advance(&initial, &k1, dt * 0.5);
        let k2 = Self::compute_derivative(body, &s2, env);
        let s3 = Self::advance(&initial, &k2, dt * 0.5);
        let k3 = Self::compute_derivative(body, &s3, env);
        let s4 = Self::advance(&initial, &k3, dt);
        let k4 = Self::compute_derivative(body, &s4, env);

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

    #[test]
    fn rk4_conserves_angular_momentum_for_torque_free_asymmetric_body() {
        // Torque-free rotation of a body with three distinct principal
        // moments of inertia. Without the gyroscopic term, ω stays constant
        // and L = Iω drifts; with it, ω precesses but ‖L‖ is conserved.
        let env = Environment::space();
        let integrator = RK4Integrator;
        let mut body = RigidBody::new(1.0).with_inertia(Vec3::new(1.0, 2.0, 3.0));
        // Initial ω misaligned with all three principal axes
        body.angular_velocity = Vec3::new(1.0, 0.5, 0.2);

        let l0 = Vec3::new(
            body.inertia.x * body.angular_velocity.x,
            body.inertia.y * body.angular_velocity.y,
            body.inertia.z * body.angular_velocity.z,
        );
        let l0_mag = l0.magnitude();
        let ke0 = body.kinetic_energy();

        let dt = 0.001;
        for _ in 0..5000 {
            integrator.step(&mut body, dt, &env);
        }

        let l = Vec3::new(
            body.inertia.x * body.angular_velocity.x,
            body.inertia.y * body.angular_velocity.y,
            body.inertia.z * body.angular_velocity.z,
        );
        let l_err = (l.magnitude() - l0_mag).abs() / l0_mag;
        let ke_err = (body.kinetic_energy() - ke0).abs() / ke0;

        assert!(
            l_err < 1e-5,
            "‖L‖ should be conserved, drift = {:.3e}",
            l_err
        );
        assert!(
            ke_err < 1e-5,
            "rotational KE should be conserved, drift = {:.3e}",
            ke_err
        );
    }
}
