use crate::point::Point;
use num_traits::Float;

/// Compute squared euclidean distance squared between two points.
///
/// Parameters:
/// - `a`: The first point.
/// - `b`: The second point.
///
/// Returns:
/// The squared euclidean distance between the two points.
pub fn euclidean_distance_squared<F: Float, const N: usize>(a: &Point<F, N>, b: &Point<F, N>) -> F {
    (a - b).norm_squared()
}

/// Compute euclidean distance between two points.
///
/// Parameters:
/// - `a`: The first point.
/// - `b`: The second point.
///
/// Returns:
/// The euclidean distance between the two points.
pub fn euclidean_distance<F: Float, const N: usize>(a: &Point<F, N>, b: &Point<F, N>) -> F {
    (a - b).norm()
}
