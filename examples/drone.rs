use physics_sandbox::{
    dynamics::RigidBody, environment::Environment, integrator::RK4Integrator, math::Vec3, World,
};

fn main() {
    let env = Environment::earth();
    let mut world = World::new(env, RK4Integrator);

    let drone = RigidBody::new(2.0)
        .with_position(Vec3::new(0.0, 0.0, 0.0))
        .with_drag(0.5, 0.1);
    let drone_id = world.add_body(drone);

    let dt = 0.01;
    let thrust = Vec3::new(0.0, 30.0, 0.0); // > 2kg * 9.81 = 19.62N

    println!("Drone simulation: 2kg drone, thrust=30N up for 2s then cut");
    println!(
        "{:<8} {:<12} {:<12}",
        "Time(s)", "Altitude(m)", "Vel_Y(m/s)"
    );
    println!("{:-<36}", "");

    let mut print_timer = 0.0;

    for step in 0..500 {
        let t = step as f64 * dt;

        if t < 2.0 {
            world.body_mut(drone_id).apply_force(thrust, None);
        }

        world.step(dt);

        print_timer += dt;
        if print_timer >= 0.1 {
            print_timer = 0.0;
            let body = world.body(drone_id);
            println!(
                "{:<8.2} {:<12.4} {:<12.4}",
                world.time(),
                body.position.y,
                body.velocity.y
            );
        }
    }
}
