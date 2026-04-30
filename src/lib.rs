pub mod collision;
pub mod dynamics;
pub mod environment;
pub mod events;
pub mod integrator;
pub mod math;

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
    speed_thresholds: Vec<(usize, f64)>,
    altitude_thresholds: Vec<(usize, f64)>,
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
        self.speed_thresholds.push((body_id, speed));
    }

    pub fn add_altitude_threshold(&mut self, body_id: usize, altitude: f64) {
        self.altitude_thresholds.push((body_id, altitude));
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

    fn check_thresholds(&self) {
        for &(body_id, threshold) in &self.speed_thresholds {
            if self.bodies[body_id].speed() > threshold {
                self.event_bus.emit(&SimEvent::Threshold {
                    body_id,
                    kind: ThresholdKind::SpeedExceeded(threshold),
                });
            }
        }
        for &(body_id, altitude) in &self.altitude_thresholds {
            if self.bodies[body_id].position.y >= altitude {
                self.event_bus.emit(&SimEvent::Threshold {
                    body_id,
                    kind: ThresholdKind::AltitudeReached(altitude),
                });
            }
        }
    }

    pub fn time(&self) -> f64 {
        self.time
    }
}
