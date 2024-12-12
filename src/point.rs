use num_traits::float::Float;
use std::ops::{Add, Div, Index, Mul, Sub};

/// Represents a Point or Vector in N-dimensional space.
/// Each dimension is represented by a generic floating-point value.
#[derive(Debug, Clone, Copy)]
pub struct Point<F: Float, const N: usize> {
    coords: [F; N],
}

impl<F: Float, const N: usize> Point<F, N> {
    /// Constructs a new point from an array of coordinates.
    ///
    /// Parameters:
    /// - `coords`: The coordinates of the point.
    ///
    /// Returns:
    /// The point.
    pub fn new(coords: [F; N]) -> Self {
        Self { coords }
    }

    /// Constructs a new point from a vector of coordinates.
    ///
    /// Parameters:
    /// - `coords`: The coordinates of the point.
    ///
    /// Returns:
    /// The point.
    pub fn from_vec(coords: Vec<F>) -> Result<Self, &'static str> {
        if coords.len() != N {
            return Err("Invalid number of coords");
        }
        let mut arr = [F::zero(); N];
        for i in 0..N {
            arr[i] = coords[i];
        }
        Ok(Self { coords: arr })
    }

    /// Returns the coordinates of the point.
    pub fn coords(&self) -> &[F; N] {
        &self.coords
    }

    /// Computes the dot product of the point with another point.
    ///
    /// Parameters:
    /// - `other`: The other point.
    ///
    /// Returns:
    /// The dot product of the two points.
    pub fn dot(&self, other: &Self) -> F {
        let mut sum = F::zero();
        for i in 0..N {
            sum = sum + self.coords[i] * other.coords[i];
        }
        sum
    }

    /// Computes the squared norm of the point. Dot product of the point with itself.
    pub fn norm_squared(&self) -> F {
        self.dot(self)
    }

    /// Computes the norm of the point.
    pub fn norm(&self) -> F {
        self.norm_squared().sqrt()
    }
}

impl<F: Float, const N: usize> Index<usize> for Point<F, N> {
    type Output = F;

    fn index(&self, index: usize) -> &Self::Output {
        &self.coords[index]
    }
}

impl<F: Float, const N: usize> Add for Point<F, N> {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        let mut coords = [F::zero(); N];
        for i in 0..N {
            coords[i] = self.coords[i] + other.coords[i];
        }
        Point::new(coords)
    }
}

impl<F: Float, const N: usize> Add for &Point<F, N> {
    type Output = Point<F, N>;

    fn add(self, other: Self) -> Point<F, N> {
        let mut coords = [F::zero(); N];
        for i in 0..N {
            coords[i] = self.coords[i] + other.coords[i];
        }
        Point::new(coords)
    }
}

impl<F: Float, const N: usize> Sub for Point<F, N> {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        let mut coords = [F::zero(); N];
        for i in 0..N {
            coords[i] = self.coords[i] - other.coords[i];
        }
        Point::new(coords)
    }
}

impl<F: Float, const N: usize> Sub for &Point<F, N> {
    type Output = Point<F, N>;

    fn sub(self, other: Self) -> Point<F, N> {
        let mut coords = [F::zero(); N];
        for i in 0..N {
            coords[i] = self.coords[i] - other.coords[i];
        }
        Point::new(coords)
    }
}

impl<F: Float, const N: usize> Mul<F> for Point<F, N> {
    type Output = Self;

    fn mul(self, scalar: F) -> Self {
        let mut coords = [F::zero(); N];
        for i in 0..N {
            coords[i] = self.coords[i] * scalar;
        }
        Point::new(coords)
    }
}

impl<F: Float, const N: usize> Mul<F> for &Point<F, N> {
    type Output = Point<F, N>;

    fn mul(self, scalar: F) -> Point<F, N> {
        let mut coords = [F::zero(); N];
        for i in 0..N {
            coords[i] = self.coords[i] * scalar;
        }
        Point::new(coords)
    }
}

impl<F: Float, const N: usize> Div<F> for Point<F, N> {
    type Output = Self;

    fn div(self, scalar: F) -> Self {
        let mut coords = [F::zero(); N];
        for i in 0..N {
            coords[i] = self.coords[i] / scalar;
        }
        Point::new(coords)
    }
}

impl<F: Float, const N: usize> Div<F> for &Point<F, N> {
    type Output = Point<F, N>;

    fn div(self, scalar: F) -> Point<F, N> {
        let mut coords = [F::zero(); N];
        for i in 0..N {
            coords[i] = self.coords[i] / scalar;
        }
        Point::new(coords)
    }
}
