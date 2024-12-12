use crate::distance::euclidean_distance_squared;
use crate::point::Point;
use num_traits::Float;

/// Checks if a point or edge is valid (i.e., not in collision).
pub trait ValidityChecker<F: Float, const N: usize> {
    /// Checks if a point is valid (i.e., does not collide with obstacles).
    ///
    /// Parameters:
    /// - `point`: The point to check.
    ///
    /// Returns:
    /// Whether the point is valid.
    fn is_point_valid(&self, point: &Point<F, N>) -> bool;

    /// Checks if an edge is valid (i.e., does not collide with obstacles).
    ///
    /// Parameters:
    /// - `a`: The start point of the edge.
    /// - `b`: The end point of the edge.
    ///
    /// Returns:
    /// Whether the edge is valid.
    fn is_edge_valid(&self, a: &Point<F, N>, b: &Point<F, N>) -> bool;
}

#[derive(Clone)]
pub struct Sphere<F: Float, const N: usize> {
    pub center: Point<F, N>,
    pub radius: F,
}

pub struct EuclideanSphericalObstacleSet<F: Float, const N: usize> {
    spheres: Vec<Sphere<F, N>>,
}

impl<F: Float, const N: usize> EuclideanSphericalObstacleSet<F, N> {
    pub fn new(spheres: Vec<Sphere<F, N>>) -> Self {
        Self { spheres }
    }

    pub fn spheres(&self) -> &Vec<Sphere<F, N>> {
        &self.spheres
    }
}

impl<F: Float, const N: usize> ValidityChecker<F, N> for EuclideanSphericalObstacleSet<F, N> {
    fn is_point_valid(&self, point: &Point<F, N>) -> bool {
        for sphere in &self.spheres {
            if euclidean_distance_squared(point, &sphere.center) <= sphere.radius * sphere.radius {
                return false;
            }
        }
        true
    }

    fn is_edge_valid(&self, a: &Point<F, N>, b: &Point<F, N>) -> bool {
        // Check if the edge intersects any sphere.

        for sphere in &self.spheres {
            // Find the closest point on the line segment ab to the sphere.
            let ab = b - a;
            let ap = sphere.center - *a;
            let t = ab.dot(&ap) / ab.norm_squared();

            // If the closest point is outside the line segment, check the endpoints.
            let radius_squared = sphere.radius * sphere.radius;
            if t < F::zero() || t > F::one() {
                if euclidean_distance_squared(a, &sphere.center) <= radius_squared
                    || euclidean_distance_squared(b, &sphere.center) <= radius_squared
                {
                    return false;
                }
                continue;
            }

            let closest = a + &(&ab * t);
            if euclidean_distance_squared(&closest, &sphere.center) <= radius_squared {
                return false;
            }
        }
        true
    }
}
