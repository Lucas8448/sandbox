//! Headless variant: runs the missile sim, records the trajectory, and
//! writes an SVG summary. Useful in CI, in docs builds, or anywhere a TTY
//! isn't available.
//!
//! Run:
//!     cargo run --example missile_svg --features viz
//!
//! Writes `docs/missile_trajectory.svg` next to the README.

use physics_sandbox::{
    World,
    dynamics::RigidBody,
    environment::Environment,
    integrator::RK4Integrator,
    math::Vec3,
    viz::{Recorder, TracePhase},
};

fn main() -> std::io::Result<()> {
    let env = Environment::new(Vec3::new(0.0, -9.81, 0.0), 1.225, Vec3::zero());
    let mut world = World::new(env, RK4Integrator);

    let angle = 45.0_f64.to_radians();
    let speed = 500.0;
    let v0 = Vec3::new(
        speed * angle.cos() * 0.95,
        speed * angle.sin(),
        speed * angle.cos() * 0.31,
    );

    let id = world.add_body(
        RigidBody::new(100.0).with_velocity(v0).with_drag(0.3, 0.5),
    );
    let thrust = v0.normalize() * 5000.0;

    let mut rec = Recorder::new(
        "Missile: 100 kg, v0 = 500 m/s @ 45°, thrust = 5 kN for 3 s",
    );

    let dt = 0.01;
    for step in 0..20_000 {
        let t = step as f64 * dt;
        let in_boost = t < 3.0;
        if in_boost {
            world.body_mut(id).apply_force(thrust, None);
        }
        world.step(dt);

        let body = world.body(id);
        if step % 5 == 0 {
            let phase = if in_boost { TracePhase::Boost } else { TracePhase::Coast };
            rec.push(world.time(), body.position, phase);
        }
        if step > 100 && body.position.y <= 0.0 {
            rec.push(
                world.time(),
                Vec3::new(body.position.x, 0.0, body.position.z),
                TracePhase::Impact,
            );
            break;
        }
    }

    let path = "docs/missile_trajectory.svg";
    std::fs::create_dir_all("docs")?;
    rec.export_svg(path)?;
    let anim_path = "docs/missile_trajectory_animated.svg";
    rec.export_svg_animated(anim_path)?;
    println!("Wrote {} and {} ({} samples)", path, anim_path, rec.samples.len());
    Ok(())
}
