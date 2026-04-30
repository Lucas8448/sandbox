//! Animated 3D terminal version of the missile example, with an orbiting
//! camera and a ground reference grid.
//!
//! Run with:
//!     cargo run --example missile_viz_3d --features viz

use std::{thread, time::Duration};

use crossterm::style::Color;
use physics_sandbox::{
    World,
    dynamics::RigidBody,
    environment::Environment,
    integrator::RK4Integrator,
    math::Vec3,
    viz::{AsciiScope, Camera, GroundGrid, Marker, prepare_terminal, restore_terminal},
};

fn main() -> std::io::Result<()> {
    let env = Environment::new(Vec3::new(0.0, -9.81, 0.0), 1.225, Vec3::zero());
    let mut world = World::new(env, RK4Integrator);

    // Same shot as missile_viz, but with a small Z component so it's not
    // perfectly axis-aligned and the 3D-ness is visible.
    let angle = 45.0_f64.to_radians();
    let speed = 500.0;
    let initial_velocity = Vec3::new(
        speed * angle.cos() * 0.95,
        speed * angle.sin(),
        speed * angle.cos() * 0.31, // ~18° off the X axis
    );

    let missile = RigidBody::new(100.0)
        .with_velocity(initial_velocity)
        .with_drag(0.3, 0.5);
    let missile_id = world.add_body(missile);

    let thrust_dir = initial_velocity.normalize();
    let thrust_force = thrust_dir * 5000.0;

    // Camera orbits this point at this radius and elevation.
    let orbit_center = Vec3::new(12_000.0, 2_500.0, 5_000.0);
    let orbit_radius = 22_000.0;
    let orbit_height = 8_000.0;

    let initial_camera = Camera::looking_at(
        orbit_center + Vec3::new(orbit_radius, orbit_height, 0.0),
        orbit_center,
    )
    .with_fov_deg(55.0);

    let mut scope = AsciiScope::new_3d(110, 34, initial_camera)
        .with_trails(true)
        .with_ground_grid(GroundGrid::new(20_000.0, 2_500.0));

    prepare_terminal()?;

    let dt = 0.01;
    let frame_every = 25; // ~30 FPS at the 33 ms sleep below
    let mut max_altitude = 0.0_f64;
    let mut landed = false;
    let mut frame_idx = 0u32;

    for step in 0..20_000 {
        let t = step as f64 * dt;

        if t < 3.0 {
            world.body_mut(missile_id).apply_force(thrust_force, None);
        }

        world.step(dt);

        let body = world.body(missile_id);
        max_altitude = max_altitude.max(body.position.y);

        if step % frame_every == 0 {
            // Orbit the camera slowly around the trajectory.
            let theta = (frame_idx as f64) * 0.02;
            let cam_eye = orbit_center
                + Vec3::new(
                    orbit_radius * theta.cos(),
                    orbit_height,
                    orbit_radius * theta.sin(),
                );
            scope.set_camera(
                Camera::looking_at(cam_eye, orbit_center).with_fov_deg(55.0),
            );

            // Bright green missile, glyph carries phase information:
            //   ▲  thrust burning   (lift-off / boost phase)
            //   ●  ballistic         (engine off, coasting)
            let (glyph, color) = if t < 3.0 {
                ('\u{25B2}', Color::Rgb { r: 60, g: 255, b: 90 })
            } else {
                ('\u{25CF}', Color::Rgb { r: 60, g: 255, b: 90 })
            };
            let marker = Marker::new3(body.position, glyph).with_color(color);

            scope.draw(&[marker])?;
            scope.hud(&format!(
                " t={:6.2}s  pos=({:7.0},{:6.0},{:6.0})  v={:6.1}m/s  thrust={}    ",
                world.time(),
                body.position.x,
                body.position.y,
                body.position.z,
                body.speed(),
                if t < 3.0 { "ON " } else { "off" },
            ))?;

            frame_idx += 1;
            thread::sleep(Duration::from_millis(33));
        }

        if step > 100 && body.position.y <= 0.0 {
            landed = true;
            break;
        }
    }

    // Final frame so the impact point is clearly visible.
    let body = world.body(missile_id);
    let final_marker = Marker::new3(
        Vec3::new(body.position.x, body.position.y.max(0.0), body.position.z),
        '\u{2739}', // ✹ a bright burst at impact
    )
    .with_color(Color::Rgb { r: 120, g: 255, b: 140 });
    scope.draw(&[final_marker])?;

    restore_terminal()?;

    println!();
    if landed {
        println!("--- IMPACT ---");
        println!("Time of flight : {:.2} s", world.time());
        println!(
            "Impact point   : ({:.1}, {:.1}, {:.1}) m",
            body.position.x, body.position.y, body.position.z
        );
        let ground_range =
            (body.position.x.powi(2) + body.position.z.powi(2)).sqrt();
        println!("Ground range   : {:.1} m", ground_range);
        println!("Max altitude   : {:.1} m", max_altitude);
        println!("Impact speed   : {:.1} m/s", body.speed());
    } else {
        println!("Simulation ended without impact (still airborne)");
    }
    Ok(())
}
