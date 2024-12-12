use crate::distance::euclidean_distance;
use crate::point::Point;
use num_traits::Float;

/// A trait for steering the robot from one point to another.
/// Allows considering the robot's kinematics and dynamics in the RRT.
pub trait Steering<F: Float, const N: usize> {
    /// Steers the robot from one point towards another.
    /// Parameters:
    /// - `from`: The start point.
    /// - `to`: The point to steer towards.
    ///
    /// Returns:
    /// The point the robot should move to.
    fn steer(&self, from: &Point<F, N>, to: &Point<F, N>) -> Point<F, N>;
}

/// A steering strategy that moves the robot in a straight line towards the goal.
pub struct EuclideanSteering<F: Float, const N: usize> {
    range: F,
}

impl<F: Float, const N: usize> Steering<F, N> for EuclideanSteering<F, N> {
    fn steer(&self, from: &Point<F, N>, to: &Point<F, N>) -> Point<F, N> {
        let distance = euclidean_distance(from, to);
        if distance <= self.range {
            to.clone()
        } else {
            let direction = to - from;
            let steering_vector = direction / distance;
            from + &(steering_vector * self.range)
        }
    }
}

impl<F: Float, const N: usize> EuclideanSteering<F, N> {
    /// Constructs a new Euclidean steering function which moves the robot in a straight line.
    /// Parameters:
    /// - `range`: The maximum distance the robot can move in one step.
    /// Returns:
    /// The Euclidean steering strategy.
    pub fn new(range: F) -> Self {
        Self { range }
    }
}
