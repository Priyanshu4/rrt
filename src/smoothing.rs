use crate::collision::ValidityChecker;
use crate::point::Point;
use num_traits::Float;

/// Smooth a path by attempting to connect nodes directly.
///
/// For each node, this attempts to connect to the furthest node that is still valid.
/// This method is not guaranteed to find the shortest path, but runs in linear time.
///
/// Parameters:
/// - `path`: The path to smooth.
/// - `validity_checker`: The validity checker used to check if edges are valid.
///
/// Returns:
/// The smoothed path.
pub fn fast_shortcutting<F: Float, const N: usize>(
    path: Vec<Point<F, N>>,
    validity_checker: &impl ValidityChecker<F, N>,
) -> Vec<Point<F, N>> {
    let mut smoothed_path = vec![path[0]];
    let mut last_valid = 0;
    for i in 1..path.len() {
        if !validity_checker.is_edge_valid(&smoothed_path[last_valid], &path[i]) {
            smoothed_path.push(path[i - 1]);
            last_valid = smoothed_path.len() - 1;
        }
    }
    smoothed_path.push(path[path.len() - 1]);
    smoothed_path
}
