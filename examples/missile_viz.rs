//! Animated terminal version of the missile example.
//!
//! Run with:
//!     cargo run --example missile_viz --features viz
//!
//! The simulation runs at the same dt as `examples/missile.rs`, but only every
//! Nth step is drawn so the animation paces nicely in a terminal.

use std::{thread, time::Duration};

use crossterm::style::Color;
use physics_sandbox::{
    World,
    dynamics::RigidBody,
    environment::Environment,
    integrator::RK4Integrator,
    math::Vec3,
    viz::{AsciiScope, Marker, prepare_terminal, restore_terminal},
};

fn main() -> std::io::Result<()> {
    let env = Environment::new(Vec3::new(0.0, -9.81, 0.0), 1.225, Vec3::zero());
    let mut world = World::new(env, RK4Integrator);

    let angle = 45.0_f64.to_radians();
    let speed = 1000.0;
    let initial_velocity = Vec3::new(speed * angle.cos(), speed * angle.sin(), 0.0);

    let missile = RigidBody::new(100.0)
        .with_velocity(initial_velocity)
        .with_drag(0.3, 0.5);
    let missile_id = world.add_body(missile);

    let thrust_dir = initial_velocity.normalize();
    let thrust_force = thrust_dir * 5000.0;

    // World extents — picked to comfortably contain a 500 m/s ballistic arc.
    let mut scope = AsciiScope::new(
        100,
        30,
        (0.0, 30_000.0),
        (0.0, 8_000.0),
    )
    .with_trails(true)
    .with_ground(true);

    prepare_terminal()?;

    let dt = 0.01;
    let frame_every = 25; // draw every 25 sim steps -> ~4 sim s per render at dt=0.01? no: 25*0.01=0.25s
    let mut max_altitude = 0.0_f64;
    let mut landed = false;

    for step in 0..20_000 {
        let t = step as f64 * dt;

        if t < 3.0 {
            world.body_mut(missile_id).apply_force(thrust_force, None);
        }

        world.step(dt);

        let body = world.body(missile_id);
        max_altitude = max_altitude.max(body.position.y);

        if step % frame_every == 0 {
            let glyph = if t < 3.0 { '^' } else { '*' };
            let color = if t < 3.0 { Color::Red } else { Color::Yellow };
            let marker = Marker::new(body.position.x, body.position.y, glyph).with_color(color);

            scope.draw(&[marker])?;
            scope.hud(&format!(
                " t={:6.2}s  x={:7.1}m  y={:7.1}m  v={:6.1}m/s  thrust={}    ",
                world.time(),
                body.position.x,
                body.position.y,
                body.speed(),
                if t < 3.0 { "ON " } else { "off" },
            ))?;

            thread::sleep(Duration::from_millis(33)); // ~30 fps
        }

        if step > 100 && body.position.y <= 0.0 {
            landed = true;
            break;
        }
    }

    // Final frame so the impact point is visible.
    let body = world.body(missile_id);
    let final_marker = Marker::new(body.position.x, body.position.y.max(0.0), 'X')
        .with_color(Color::Magenta);
    scope.draw(&[final_marker])?;

    restore_terminal()?;

    println!();
    if landed {
        println!("--- IMPACT ---");
        println!("Time of flight : {:.2} s", world.time());
        println!("Range          : {:.1} m", body.position.x);
        println!("Max altitude   : {:.1} m", max_altitude);
        println!("Impact speed   : {:.1} m/s", body.speed());
    } else {
        println!("Simulation ended without impact (still airborne)");
    }
    Ok(())
}
