use crate::collision::CollisionResult;

#[derive(Debug, Clone)]
pub enum ThresholdKind {
    SpeedExceeded(f64),
    AltitudeReached(f64),
    Custom(String),
}

#[derive(Debug, Clone)]
pub enum SimEvent {
    Collision {
        body_a: usize,
        body_b: usize,
        result: CollisionResult,
    },
    Threshold {
        body_id: usize,
        kind: ThresholdKind,
    },
}

pub struct EventBus {
    callbacks: Vec<Box<dyn Fn(&SimEvent)>>,
}

impl EventBus {
    pub fn new() -> Self {
        Self {
            callbacks: Vec::new(),
        }
    }

    pub fn subscribe(&mut self, callback: impl Fn(&SimEvent) + 'static) {
        self.callbacks.push(Box::new(callback));
    }

    pub fn emit(&self, event: &SimEvent) {
        for cb in &self.callbacks {
            cb(event);
        }
    }
}

impl Default for EventBus {
    fn default() -> Self {
        Self::new()
    }
}
