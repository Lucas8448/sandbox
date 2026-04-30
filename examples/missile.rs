use physics_sandbox::{
    dynamics::RigidBody, environment::Environment, integrator::RK4Integrator, math::Vec3, World,
};

fn main() {
    let env = Environment::new(Vec3::new(0.0, -9.81, 0.0), 1.225, Vec3::zero());
    let mut world = World::new(env, RK4Integrator);

    let angle = 45.0_f64.to_radians();
    let speed = 500.0;
    let initial_velocity = Vec3::new(speed * angle.cos(), speed * angle.sin(), 0.0);

    let missile = RigidBody::new(100.0)
        .with_velocity(initial_velocity)
        .with_drag(0.3, 0.5);
    let missile_id = world.add_body(missile);

    let thrust_dir = initial_velocity.normalize();
    let thrust_force = thrust_dir * 5000.0;

    let dt = 0.01;
    let mut max_altitude = 0.0_f64;
    let mut landed = false;

    println!("Missile simulation: 100kg, v0=500m/s @ 45°, thrust=5000N for 3s");
    println!(
        "{:<8} {:<12} {:<12} {:<12}",
        "Time(s)", "Range(m)", "Alt(m)", "Speed(m/s)"
    );
    println!("{:-<48}", "");

    let mut print_timer = 0.0;

    for step in 0..20000 {
        let t = step as f64 * dt;

        if t < 3.0 {
            world.body_mut(missile_id).apply_force(thrust_force, None);
        }

        world.step(dt);

        let body = world.body(missile_id);
        max_altitude = max_altitude.max(body.position.y);

        print_timer += dt;
        if print_timer >= 5.0 {
            print_timer = 0.0;
            println!(
                "{:<8.2} {:<12.1} {:<12.1} {:<12.1}",
                world.time(),
                body.position.x,
                body.position.y,
                body.speed()
            );
        }

        if step > 100 && body.position.y <= 0.0 {
            println!("\n--- IMPACT ---");
            println!("Time of flight: {:.2}s", world.time());
            println!("Range: {:.1}m", body.position.x);
            println!("Max altitude: {:.1}m", max_altitude);
            println!("Impact speed: {:.1}m/s", body.speed());
            landed = true;
            break;
        }
    }

    if !landed {
        println!("\nSimulation ended without impact (still airborne)");
    }
}
