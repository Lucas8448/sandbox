//! Five drones flying in a V formation. The whole formation cruises along
//! +X at altitude 100 m for 12 s, then performs a coordinated right-hand
//! banking turn for the rest of the run.
//!
//! Each drone is a 2 kg quad with quadratic drag. Per step we apply:
//!
//!   - gravity compensation: m*g upward
//!   - a PD position controller toward the drone's target slot in the
//!     (translating + rotating) formation frame
//!
//! The trajectories are recorded into a `MultiRecorder` and exported to
//! `docs/drones_formation.svg` — a 4-up SVG with the same SIDE / TOP /
//! FRONT layout as `missile_svg`, plus a legend.
//!
//! Run:
//!     cargo run --example drones_formation --features viz

use physics_sandbox::{
    World,
    dynamics::RigidBody,
    environment::Environment,
    integrator::RK4Integrator,
    math::Vec3,
    viz::MultiRecorder,
};

const ALTITUDE: f64 = 100.0;
const CRUISE_SPEED: f64 = 15.0; // m/s along +X initially
const TURN_START: f64 = 12.0;   // s
const TURN_OMEGA: f64 = 0.06;   // rad/s, right-hand turn (toward +Z)
const TOTAL_TIME: f64 = 40.0;
const DT: f64 = 0.02;
const MASS: f64 = 2.0;

/// Heading angle (radians, around +Y) and leader displacement at time `t`.
fn leader_state(t: f64) -> (f64, Vec3) {
    if t < TURN_START {
        (0.0, Vec3::new(CRUISE_SPEED * t, 0.0, 0.0))
    } else {
        let r = CRUISE_SPEED / TURN_OMEGA;
        let theta = (t - TURN_START) * TURN_OMEGA;
        // Right-hand turn: center is offset +Z from the pre-turn endpoint.
        // Path is a circle of radius r tangent to +X at (CRUISE_SPEED*TURN_START, 0).
        let pre_x = CRUISE_SPEED * TURN_START;
        let dx = r * theta.sin();
        let dz = r * (1.0 - theta.cos());
        (theta, Vec3::new(pre_x + dx, 0.0, dz))
    }
}

/// Target position for a drone whose body-frame slot is `slot_offset`
/// (offset from the leader, in the formation's local X/Z frame, with Y
/// being the altitude offset from `ALTITUDE`).
fn target_pos(slot_offset: Vec3, t: f64) -> Vec3 {
    let (heading, leader_disp) = leader_state(t);
    let cos_h = heading.cos();
    let sin_h = heading.sin();
    let rotated = Vec3::new(
        slot_offset.x * cos_h - slot_offset.z * sin_h,
        slot_offset.y,
        slot_offset.x * sin_h + slot_offset.z * cos_h,
    );
    Vec3::new(0.0, ALTITUDE, 0.0) + leader_disp + rotated
}

fn main() -> std::io::Result<()> {
    let env = Environment::new(Vec3::new(0.0, -9.81, 0.0), 1.225, Vec3::zero());
    let mut world = World::new(env, RK4Integrator);

    // V formation, slot offsets in the leader's body frame (-X = behind).
    let slots: [(Vec3, &str, &str); 5] = [
        (Vec3::new(  0.0, 0.0,  0.0), "Lead",   "#5cff90"),
        (Vec3::new(-15.0, 0.0, -8.0), "Wing-L", "#5c9aff"),
        (Vec3::new(-15.0, 0.0,  8.0), "Wing-R", "#ff5c5c"),
        (Vec3::new(-30.0, 0.0, -16.0), "Tail-L", "#ffd45c"),
        (Vec3::new(-30.0, 0.0,  16.0), "Tail-R", "#c45cff"),
    ];

    let mut ids = Vec::new();
    let mut rec = MultiRecorder::new(
        "5 drones (2 kg, quad-drag) in V formation @ 15 m/s, banking right at t=12 s",
    );
    let mut track_idx = Vec::new();

    for (slot, label, color) in &slots {
        let p0 = target_pos(*slot, 0.0);
        let v0 = (target_pos(*slot, DT) - p0) * (1.0 / DT);
        let body = RigidBody::new(MASS)
            .with_position(p0)
            .with_velocity(v0)
            .with_drag(0.05, 0.4);
        ids.push(world.add_body(body));
        track_idx.push(rec.add_track(*label, *color));
    }

    let kp = 4.0; // position gain
    let kd = 2.5; // velocity gain
    let steps = (TOTAL_TIME / DT) as usize;

    for step in 0..steps {
        let t = step as f64 * DT;

        for (i, &id) in ids.iter().enumerate() {
            let slot = slots[i].0;
            let target = target_pos(slot, t);
            let target_next = target_pos(slot, t + DT);
            let target_vel = (target_next - target) * (1.0 / DT);

            let body = world.body(id);
            let pos_err = target - body.position;
            let vel_err = target_vel - body.velocity;

            let f = pos_err * (MASS * kp)
                + vel_err * (MASS * kd)
                + Vec3::new(0.0, MASS * 9.81, 0.0); // gravity comp

            world.body_mut(id).apply_force(f, None);
        }

        world.step(DT);

        // Sample 5x/s — dense enough for smooth curves, light on file size.
        if step % 10 == 0 {
            for (i, &id) in ids.iter().enumerate() {
                rec.push(track_idx[i], world.time(), world.body(id).position);
            }
        }
    }

    // Final sample so the end markers sit at the actual stop point.
    for (i, &id) in ids.iter().enumerate() {
        rec.push(track_idx[i], world.time(), world.body(id).position);
    }

    let path = "docs/drones_formation.svg";
    std::fs::create_dir_all("docs")?;
    rec.export_svg(path)?;
    println!(
        "Wrote {} ({} drones, {:.0} s, {} total samples)",
        path,
        rec.tracks.len(),
        world.time(),
        rec.tracks.iter().map(|t| t.samples.len()).sum::<usize>(),
    );
    Ok(())
}
