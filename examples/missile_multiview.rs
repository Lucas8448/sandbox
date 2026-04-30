//! Multi-view (SIDE / TOP / FRONT) live ASCII visualization, plus an SVG
//! summary written to disk after the run.
//!
//! Run:
//!     cargo run --example missile_multiview --features viz
//!
//! Produces `missile_trajectory.svg` in the current directory at the end.

use std::{thread, time::Duration};

use crossterm::style::Color;
use physics_sandbox::{
    World,
    dynamics::RigidBody,
    environment::Environment,
    integrator::RK4Integrator,
    math::Vec3,
    viz::{
        Marker, MultiView, Recorder, TracePhase, prepare_terminal, restore_terminal,
    },
};

fn main() -> std::io::Result<()> {
    let env = Environment::new(Vec3::new(0.0, -9.81, 0.0), 1.225, Vec3::zero());
    let mut world = World::new(env, RK4Integrator);

    // 500 m/s @ 45° elevation, 18° off the X axis (so Z is non-trivial).
    let angle = 45.0_f64.to_radians();
    let speed = 500.0;
    let initial_velocity = Vec3::new(
        speed * angle.cos() * 0.95,
        speed * angle.sin(),
        speed * angle.cos() * 0.31,
    );

    let missile = RigidBody::new(100.0)
        .with_velocity(initial_velocity)
        .with_drag(0.3, 0.5);
    let missile_id = world.add_body(missile);

    let thrust_dir = initial_velocity.normalize();
    let thrust_force = thrust_dir * 5000.0;

    // Each panel is 36 cols × 16 rows. Three panels + 2 char gaps = 112 cols.
    // Shrink panel_w if your terminal is narrower.
    let mut cluster = MultiView::three_view(
        36,
        16,
        (0.0, 30_000.0),  // X (range)
        (0.0, 8_000.0),   // Y (altitude)
        (-2_000.0, 12_000.0), // Z (cross-range)
    );

    let mut recorder = Recorder::new(
        "Missile: 100 kg, v0 = 500 m/s @ 45°, thrust = 5 kN for 3 s",
    );

    prepare_terminal()?;

    let dt = 0.01;
    let frame_every = 25;
    let record_every = 5; // sample SVG path more densely than we render
    let mut max_altitude = 0.0_f64;
    let mut landed = false;

    for step in 0..20_000 {
        let t = step as f64 * dt;
        let in_boost = t < 3.0;

        if in_boost {
            world.body_mut(missile_id).apply_force(thrust_force, None);
        }

        world.step(dt);

        let body = world.body(missile_id);
        max_altitude = max_altitude.max(body.position.y);

        if step % record_every == 0 {
            let phase = if in_boost { TracePhase::Boost } else { TracePhase::Coast };
            recorder.push(world.time(), body.position, phase);
        }

        if step % frame_every == 0 {
            // Bright green missile, glyph carries phase.
            let (glyph, color) = if in_boost {
                ('^', Color::Rgb { r: 60, g: 255, b: 90 })
            } else {
                ('o', Color::Rgb { r: 60, g: 255, b: 90 })
            };
            let marker = Marker::new3(body.position, glyph).with_color(color);

            cluster.draw(&[marker])?;
            cluster.hud(&format!(
                " t={:6.2}s  pos=({:7.0},{:6.0},{:6.0})  v={:6.1}m/s  thrust={}    ",
                world.time(),
                body.position.x,
                body.position.y,
                body.position.z,
                body.speed(),
                if in_boost { "ON " } else { "off" },
            ))?;

            thread::sleep(Duration::from_millis(33));
        }

        if step > 100 && body.position.y <= 0.0 {
            landed = true;
            break;
        }
    }

    // Final frame + impact sample.
    let body = world.body(missile_id);
    let impact_pos = Vec3::new(body.position.x, body.position.y.max(0.0), body.position.z);
    recorder.push(world.time(), impact_pos, TracePhase::Impact);

    let final_marker = Marker::new3(impact_pos, 'X')
        .with_color(Color::Rgb { r: 120, g: 255, b: 140 });
    cluster.draw(&[final_marker])?;

    restore_terminal()?;

    println!();
    if landed {
        let ground_range = (body.position.x.powi(2) + body.position.z.powi(2)).sqrt();
        println!("--- IMPACT ---");
        println!("Time of flight : {:.2} s", world.time());
        println!(
            "Impact point   : ({:.1}, {:.1}, {:.1}) m",
            body.position.x, body.position.y, body.position.z
        );
        println!("Ground range   : {:.1} m", ground_range);
        println!("Max altitude   : {:.1} m", max_altitude);
        println!("Impact speed   : {:.1} m/s", body.speed());
    } else {
        println!("Simulation ended without impact (still airborne)");
    }

    let svg_path = "missile_trajectory.svg";
    recorder.export_svg(svg_path)?;
    println!("\nSVG summary written to {}", svg_path);

    Ok(())
}
