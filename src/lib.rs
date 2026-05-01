pub mod collision;
pub mod dynamics;
pub mod environment;
pub mod events;
pub mod integrator;
pub mod math;

#[cfg(feature = "viz")]
pub mod viz;

use collision::sphere_sphere;
use dynamics::RigidBody;
use environment::Environment;
use events::{EventBus, SimEvent, ThresholdKind};
use integrator::Integrator;

pub struct World {
    pub bodies: Vec<RigidBody>,
    pub environment: Environment,
    integrator: Box<dyn Integrator>,
    pub event_bus: EventBus,
    pub time: f64,
    collider_radii: Vec<f64>,
    // Thresholds are edge-triggered: an event fires only on the step the
    // value crosses the threshold, not every step it stays past it.
    speed_thresholds: Vec<(usize, f64)>,
    altitude_thresholds: Vec<(usize, f64)>,
    speed_above: Vec<bool>,      // parallel to speed_thresholds
    altitude_above: Vec<bool>,   // parallel to altitude_thresholds
}

impl World {
    pub fn new(environment: Environment, integrator: impl Integrator + 'static) -> Self {
        Self {
            bodies: Vec::new(),
            environment,
            integrator: Box::new(integrator),
            event_bus: EventBus::new(),
            time: 0.0,
            collider_radii: Vec::new(),
            speed_thresholds: Vec::new(),
            altitude_thresholds: Vec::new(),
            speed_above: Vec::new(),
            altitude_above: Vec::new(),
        }
    }

    pub fn add_body(&mut self, body: RigidBody) -> usize {
        let id = self.bodies.len();
        self.bodies.push(body);
        self.collider_radii.push(0.0);
        id
    }

    pub fn set_collider_radius(&mut self, id: usize, radius: f64) {
        self.collider_radii[id] = radius;
    }

    pub fn add_speed_threshold(&mut self, body_id: usize, speed: f64) {
        let already = self.bodies[body_id].speed() > speed;
        self.speed_thresholds.push((body_id, speed));
        self.speed_above.push(already);
    }

    pub fn add_altitude_threshold(&mut self, body_id: usize, altitude: f64) {
        let already = self.bodies[body_id].position.y >= altitude;
        self.altitude_thresholds.push((body_id, altitude));
        self.altitude_above.push(already);
    }

    pub fn body(&self, id: usize) -> &RigidBody {
        &self.bodies[id]
    }

    pub fn body_mut(&mut self, id: usize) -> &mut RigidBody {
        &mut self.bodies[id]
    }

    pub fn step(&mut self, dt: f64) {
        for body in &mut self.bodies {
            self.integrator.step(body, dt, &self.environment);
        }

        self.check_collisions();
        self.check_thresholds();
        self.time += dt;
    }

    fn check_collisions(&self) {
        let n = self.bodies.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let ri = self.collider_radii[i];
                let rj = self.collider_radii[j];
                if ri <= 0.0 || rj <= 0.0 {
                    continue;
                }
                let result =
                    sphere_sphere(self.bodies[i].position, ri, self.bodies[j].position, rj);
                if result.hit {
                    self.event_bus.emit(&SimEvent::Collision {
                        body_a: i,
                        body_b: j,
                        result,
                    });
                }
            }
        }
    }

    fn check_thresholds(&mut self) {
        for (i, &(body_id, threshold)) in self.speed_thresholds.iter().enumerate() {
            let now_above = self.bodies[body_id].speed() > threshold;
            if now_above && !self.speed_above[i] {
                self.event_bus.emit(&SimEvent::Threshold {
                    body_id,
                    kind: ThresholdKind::SpeedExceeded(threshold),
                });
            }
            self.speed_above[i] = now_above;
        }
        for (i, &(body_id, altitude)) in self.altitude_thresholds.iter().enumerate() {
            let now_above = self.bodies[body_id].position.y >= altitude;
            if now_above && !self.altitude_above[i] {
                self.event_bus.emit(&SimEvent::Threshold {
                    body_id,
                    kind: ThresholdKind::AltitudeReached(altitude),
                });
            }
            self.altitude_above[i] = now_above;
        }
    }

    pub fn time(&self) -> f64 {
        self.time
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::RigidBody;
    use crate::events::{SimEvent, ThresholdKind};
    use crate::integrator::EulerIntegrator;
    use crate::math::Vec3;
    use std::sync::{Arc, Mutex};

    #[test]
    fn altitude_threshold_is_edge_triggered() {
        // Body lobbed straight up — crosses 5m on the way up, again on the way down.
        // With edge triggering we should see exactly one event per upward crossing.
        let env = Environment::earth();
        let mut world = World::new(env, EulerIntegrator);
        let id = world.add_body(
            RigidBody::new(1.0)
                .with_position(Vec3::zero())
                .with_velocity(Vec3::new(0.0, 12.0, 0.0)),
        );
        world.add_altitude_threshold(id, 5.0);

        let count = Arc::new(Mutex::new(0u32));
        let c = count.clone();
        world.event_bus.subscribe(move |e| {
            if let SimEvent::Threshold {
                kind: ThresholdKind::AltitudeReached(_),
                ..
            } = e
            {
                *c.lock().unwrap() += 1;
            }
        });

        let dt = 0.001;
        for _ in 0..3000 {
            world.step(dt);
        }

        let n = *count.lock().unwrap();
        assert_eq!(n, 1, "expected exactly one altitude event, got {}", n);
    }
}
