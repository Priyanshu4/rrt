use crate::distance::euclidean_distance_squared;
use crate::point::Point;
use kiddo::float::{distance::SquaredEuclidean, kdtree::Axis, kdtree::KdTree};
use num_traits::Float;

/// A trait for a nearest neighbor data structure that supports nearest neighbors and radius queries.
/// Stores points and a usize index along with them.
pub trait NearestNeighbors<F: Float, const N: usize> {
    /// Constructs a new nearest neighbor data structure.
    /// The data structure is empty initially.
    fn new() -> Self;

    /// Adds a point to the data structure.
    ///
    /// Parameters:
    /// - `point`: The point to add.
    /// - `item`: The index of the point.
    fn add(&mut self, point: Point<F, N>, item: usize);

    /// Gets the nearest neighbor to the given point.
    ///
    /// Parameters:
    /// - `point`: The point to find the nearest neighbor to.
    ///
    /// Returns:
    /// The item/index of the nearest neighbor, if any.
    fn nearest_one(&self, point: &Point<F, N>) -> Option<usize> {
        let nearest_vec = self.nearest_k(point, 1);
        if nearest_vec.is_empty() {
            None
        } else {
            Some(nearest_vec[0])
        }
    }

    /// Gets the k nearest neighbors to the given point.
    ///
    /// Parameters:
    /// - `point`: The point to find the nearest neighbors to.
    /// - `k`: The number of neighbors to find.
    ///
    /// Returns:
    /// The items/indices of the k nearest neighbors.
    fn nearest_k(&self, point: &Point<F, N>, k: usize) -> Vec<usize>;

    /// Gets all points within a given radius of the given point.
    ///
    /// Parameters:
    /// - `point`: The point to find the neighbors of.
    /// - `radius`: The radius within which to find neighbors.
    ///
    /// Returns:
    /// The items/indices of the points within the radius.
    fn within_radius(&self, point: &Point<F, N>, radius: F) -> Vec<usize>;
}

/// A nearest neighbor data structure that uses a linear search to find the nearest neighbors.
/// This is useful for small datasets.
pub struct LinearNearestNeighbors<F: Float, const N: usize> {
    points: Vec<(Point<F, N>, usize)>,
}

impl<F: Float, const N: usize> NearestNeighbors<F, N> for LinearNearestNeighbors<F, N> {
    fn new() -> Self {
        Self { points: Vec::new() }
    }

    fn add(&mut self, point: Point<F, N>, item: usize) {
        self.points.push((point, item));
    }

    fn nearest_one(&self, point: &Point<F, N>) -> Option<usize> {
        let nearest = self.points.iter().min_by(|a, b| {
            euclidean_distance_squared(&a.0, point)
                .partial_cmp(&euclidean_distance_squared(&b.0, point))
                .unwrap()
        });
        nearest.map(|(_, i)| *i)
    }

    fn nearest_k(&self, point: &Point<F, N>, k: usize) -> Vec<usize> {
        let mut nearest = self
            .points
            .iter()
            .map(|(p, i)| (euclidean_distance_squared(&p, &point), *i))
            .collect::<Vec<_>>();
        nearest.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        nearest.into_iter().take(k).map(|(_, i)| i).collect()
    }

    fn within_radius(&self, point: &Point<F, N>, radius: F) -> Vec<usize> {
        self.points
            .iter()
            .filter(|(p, _)| euclidean_distance_squared(&p, &point) <= radius * radius)
            .map(|(_, i)| *i)
            .collect()
    }
}

pub struct KdTreeNearestNeighbors<F: Float + Axis, const N: usize> {
    kdtree: KdTree<F, usize, N, 32, u32>,
}

impl<F: Float + Axis, const N: usize> NearestNeighbors<F, N> for KdTreeNearestNeighbors<F, N> {
    fn new() -> Self {
        Self {
            kdtree: KdTree::new(),
        }
    }

    fn add(&mut self, point: Point<F, N>, item: usize) {
        self.kdtree.add(point.coords(), item);
    }

    fn nearest_one(&self, point: &Point<F, N>) -> Option<usize> {
        let neighbor = self.kdtree.nearest_one::<SquaredEuclidean>(point.coords());
        Some(neighbor.item)
    }

    fn nearest_k(&self, point: &Point<F, N>, k: usize) -> Vec<usize> {
        self.kdtree
            .nearest_n::<SquaredEuclidean>(point.coords(), k)
            .iter()
            .map(|n| n.item)
            .collect()
    }

    fn within_radius(&self, point: &Point<F, N>, radius: F) -> Vec<usize> {
        self.kdtree
            .within::<SquaredEuclidean>(point.coords(), radius * radius)
            .iter()
            .map(|n| n.item)
            .collect()
    }
}
