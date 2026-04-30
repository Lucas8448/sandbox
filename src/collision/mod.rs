use crate::math::Vec3;

#[derive(Debug, Clone)]
pub enum Collider {
    Sphere { center: Vec3, radius: f64 },
    AABB { center: Vec3, half_extents: Vec3 },
    Ray { origin: Vec3, direction: Vec3 },
}

#[derive(Debug, Clone)]
pub struct CollisionResult {
    pub hit: bool,
    pub point: Option<Vec3>,
    pub normal: Option<Vec3>,
    pub penetration: f64,
}

impl CollisionResult {
    pub fn miss() -> Self {
        Self {
            hit: false,
            point: None,
            normal: None,
            penetration: 0.0,
        }
    }

    pub fn new(point: Vec3, normal: Vec3, penetration: f64) -> Self {
        Self {
            hit: true,
            point: Some(point),
            normal: Some(normal),
            penetration,
        }
    }
}

pub fn sphere_sphere(c1: Vec3, r1: f64, c2: Vec3, r2: f64) -> CollisionResult {
    let diff = c2 - c1;
    let dist = diff.magnitude();
    let sum_r = r1 + r2;

    if dist >= sum_r {
        return CollisionResult::miss();
    }

    let normal = if dist > 1e-12 {
        diff.normalize()
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let penetration = sum_r - dist;
    let point = c1 + normal * (r1 - penetration * 0.5);

    CollisionResult::new(point, normal, penetration)
}

pub fn sphere_aabb(
    sphere_center: Vec3,
    radius: f64,
    aabb_center: Vec3,
    half: Vec3,
) -> CollisionResult {
    // Find closest point on AABB to sphere center
    let closest = Vec3::new(
        clamp(
            sphere_center.x,
            aabb_center.x - half.x,
            aabb_center.x + half.x,
        ),
        clamp(
            sphere_center.y,
            aabb_center.y - half.y,
            aabb_center.y + half.y,
        ),
        clamp(
            sphere_center.z,
            aabb_center.z - half.z,
            aabb_center.z + half.z,
        ),
    );

    let diff = sphere_center - closest;
    let dist_sq = diff.magnitude_sq();

    if dist_sq >= radius * radius {
        return CollisionResult::miss();
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-12 {
        diff.normalize()
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let penetration = radius - dist;

    CollisionResult::new(closest, normal, penetration)
}

pub fn ray_sphere(origin: Vec3, direction: Vec3, center: Vec3, radius: f64) -> CollisionResult {
    let dir = direction.normalize();
    let oc = origin - center;
    let b = oc.dot(dir);
    let c = oc.dot(oc) - radius * radius;
    let discriminant = b * b - c;

    if discriminant < 0.0 {
        return CollisionResult::miss();
    }

    let t = -b - discriminant.sqrt();
    if t < 0.0 {
        // Try other root
        let t2 = -b + discriminant.sqrt();
        if t2 < 0.0 {
            return CollisionResult::miss();
        }
        let point = origin + dir * t2;
        let normal = (point - center).normalize();
        return CollisionResult::new(point, normal, 0.0);
    }

    let point = origin + dir * t;
    let normal = (point - center).normalize();
    CollisionResult::new(point, normal, 0.0)
}

pub fn ray_aabb(origin: Vec3, direction: Vec3, aabb_center: Vec3, half: Vec3) -> CollisionResult {
    let min = aabb_center - half;
    let max = aabb_center + half;

    let inv_dir = Vec3::new(
        if direction.x.abs() > 1e-12 {
            1.0 / direction.x
        } else {
            f64::INFINITY
        },
        if direction.y.abs() > 1e-12 {
            1.0 / direction.y
        } else {
            f64::INFINITY
        },
        if direction.z.abs() > 1e-12 {
            1.0 / direction.z
        } else {
            f64::INFINITY
        },
    );

    let t1 = (min.x - origin.x) * inv_dir.x;
    let t2 = (max.x - origin.x) * inv_dir.x;
    let t3 = (min.y - origin.y) * inv_dir.y;
    let t4 = (max.y - origin.y) * inv_dir.y;
    let t5 = (min.z - origin.z) * inv_dir.z;
    let t6 = (max.z - origin.z) * inv_dir.z;

    let tmin = t1.min(t2).max(t3.min(t4)).max(t5.min(t6));
    let tmax = t1.max(t2).min(t3.max(t4)).min(t5.max(t6));

    if tmax < 0.0 || tmin > tmax {
        return CollisionResult::miss();
    }

    let t = if tmin >= 0.0 { tmin } else { tmax };
    let point = origin + direction * t;

    // Determine hit normal
    let p_local = point - aabb_center;
    let normal = if (p_local.x.abs() - half.x).abs() < 1e-6 {
        Vec3::new(p_local.x.signum(), 0.0, 0.0)
    } else if (p_local.y.abs() - half.y).abs() < 1e-6 {
        Vec3::new(0.0, p_local.y.signum(), 0.0)
    } else {
        Vec3::new(0.0, 0.0, p_local.z.signum())
    };

    CollisionResult::new(point, normal, 0.0)
}

fn clamp(val: f64, min: f64, max: f64) -> f64 {
    val.max(min).min(max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_sphere_collision() {
        let r = sphere_sphere(Vec3::zero(), 1.0, Vec3::new(1.5, 0.0, 0.0), 1.0);
        assert!(r.hit);
        assert!((r.penetration - 0.5).abs() < 1e-10);
    }

    #[test]
    fn sphere_sphere_miss() {
        let r = sphere_sphere(Vec3::zero(), 1.0, Vec3::new(3.0, 0.0, 0.0), 1.0);
        assert!(!r.hit);
    }

    #[test]
    fn ray_hits_sphere() {
        let r = ray_sphere(
            Vec3::new(-5.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::zero(),
            1.0,
        );
        assert!(r.hit);
        let p = r.point.unwrap();
        assert!((p.x - (-1.0)).abs() < 1e-10);
    }
}
