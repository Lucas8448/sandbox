# physics_sandbox

A small, dependency-free rigid-body physics sandbox in Rust. Pluggable integrators, environment forces (gravity, wind, air density), simple sphere–sphere collisions, and a tiny event bus for collisions and threshold triggers. Optional retro ASCII terminal visualization behind the `viz` feature.

## Add it

```toml
[dependencies]
physics_sandbox = "0.1"

# with the terminal visualization helpers:
physics_sandbox = { version = "0.1", features = ["viz"] }
```

## Quick start

```rust
use physics_sandbox::{
    World,
    dynamics::RigidBody,
    environment::Environment,
    integrator::RK4Integrator,
    math::Vec3,
};

let env = Environment::new(Vec3::new(0.0, -9.81, 0.0), 1.225, Vec3::zero());
let mut world = World::new(env, RK4Integrator);

let id = world.add_body(
    RigidBody::new(100.0)
        .with_velocity(Vec3::new(354.0, 354.0, 0.0)) // 500 m/s @ 45°
        .with_drag(0.3, 0.5),
);

for _ in 0..10_000 {
    world.step(0.01);
    if world.body(id).position.y <= 0.0 { break; }
}
```

## Examples

```bash
cargo run --example missile
cargo run --example drone
cargo run --example missile_viz --features viz   # animated ASCII scope
```

## Features

- `viz` — `AsciiScope` + `Marker` for animating bodies in the terminal via `crossterm`.

## License

Dual-licensed under either of:

- MIT license ([LICENSE-MIT](LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))

at your option.
