use crate::collision::ValidityChecker;
use crate::distance::euclidean_distance_squared;
use crate::neighbors::NearestNeighbors;
use crate::point::Point;
use crate::sampling::SamplingDistribution;
use crate::steering::Steering;
use num_traits::Float;

/// A node in the RRT tree.
#[derive(Clone)]
pub struct Node<F: Float, const N: usize> {
    /// The point in N-dimensional space.
    point: Point<F, N>,
    /// The index of the parent node (None if the node is the root).
    parent: Option<usize>,
}

impl<F: Float, const N: usize> Node<F, N> {
    /// Constructs a new node.
    /// Parameters:
    /// - `point`: The point in N-dimensional space.
    /// - `parent`: The index of the parent node (None if the node is the root).
    pub fn new(point: Point<F, N>, parent: Option<usize>) -> Self {
        Self { point, parent }
    }

    pub fn point(&self) -> &Point<F, N> {
        &self.point
    }

    pub fn parent(&self) -> Option<usize> {
        self.parent
    }
}

/// A Rapidly-exploring Random Tree (RRT) planner.
/// Template Parameters:
/// - `F`: The floating-point type.
/// - `N`: The dimension of the space.
/// - `VC`: The validity checker.
/// - `SD`: The sampling distribution.
/// - `ST`: The steering function.
/// - `NN`: The nearest neighbors data structure.
pub struct RRT<F: Float, const N: usize, VC, SD, ST, NN>
where
    VC: ValidityChecker<F, N>,
    SD: SamplingDistribution<F, N>,
    ST: Steering<F, N>,
    NN: NearestNeighbors<F, N>,
{
    /// The goal state.
    goal: Point<F, N>,
    /// The tolerance for reaching the goal.
    goal_tolerance: F,
    /// The nodes in the tree.
    nodes: Vec<Node<F, N>>,
    /// Index of the solution node (None if no solution has been found).
    solution: Option<usize>,
    validity_checker: VC,
    sampling_distribution: SD,
    steering: ST,
    nearest_neighbors: NN,
}

impl<F: Float, const N: usize, VC, SD, ST, NN> RRT<F, N, VC, SD, ST, NN>
where
    VC: ValidityChecker<F, N>,
    SD: SamplingDistribution<F, N>,
    ST: Steering<F, N>,
    NN: NearestNeighbors<F, N>,
{
    /// Constructs a new RRT planner.
    ///
    /// Parameters:
    /// - `start`: The start point.
    /// - `goal`: The goal point.
    /// - `goal_tolerance`: The tolerance for reaching the goal.
    /// - `validity_checker`: Checks if the edges or nodes as valid.
    /// - `sampling_distribution`: The sampling distribution.
    /// - `steering`: The steering function.
    /// Returns the RRT planner.
    pub fn new(
        start: Point<F, N>,
        goal: Point<F, N>,
        goal_tolerance: F,
        validity_checker: VC,
        sampling_distribution: SD,
        steering: ST,
    ) -> Self {
        let mut rrt = Self {
            goal,
            goal_tolerance,
            solution: None,
            nodes: Vec::new(),
            validity_checker,
            sampling_distribution,
            steering,
            nearest_neighbors: NN::new(),
        };
        let root = Node::new(start, None);
        rrt.add_node(root);
        rrt
    }

    /// Attempts to find a solution within a maximum number of iterations.
    ///
    /// Terminates and returns true when a solution is found. Otherwise, returns false.
    ///
    /// Parameters:
    /// - `max_iterations`: The maximum number of iterations.
    pub fn solve(&mut self, max_iterations: u32) -> bool {
        for _ in 0..max_iterations {
            self.iteration();
            if self.solved() {
                return true;
            }
        }
        return false;
    }

    /// Run a fixed number of iterations of the RRT algorithm. Does not terminate early if a solution is found.
    ///
    /// Returns true if the RRT found a solution.
    ///
    /// Parameters:
    /// - `iterations`: The number of iterations to run.
    pub fn run_iterations(&mut self, iterations: u32) -> bool {
        for _ in 0..iterations {
            self.iteration();
            if self.solved() {
                return true;
            }
        }
        return false;
    }

    /// Returns true if a solution was found.
    pub fn solved(&self) -> bool {
        self.solution.is_some()
    }

    /// Returns the path from the start to the goal, if a solution was found.
    pub fn get_path(&self) -> Option<Vec<Point<F, N>>> {
        if !self.solved() {
            return None;
        }

        let mut path = Vec::new();
        let mut current_index = self.solution.unwrap();

        // Reconstruct the path by backtracking up the tree (following the parent pointers).
        while let Some(parent_index) = self.nodes[current_index].parent {
            path.push(self.nodes[current_index].point);
            current_index = parent_index;
        }
        path.push(self.nodes[current_index].point);

        // Reverse the path so that it goes from the start to the goal.
        path.reverse();
        Some(path)
    }

    pub fn get_nearest_neighbors(&self) -> &NN {
        &self.nearest_neighbors
    }

    pub fn get_sampling_distribution(&self) -> &SD {
        &self.sampling_distribution
    }

    pub fn get_validity_checker(&self) -> &VC {
        &self.validity_checker
    }

    /// Returns the vector of nodes in the tree.
    pub fn get_tree(&self) -> &Vec<Node<F, N>> {
        &self.nodes
    }

    /// Expands the tree by one iteration.
    ///
    /// Each iteration of the RRT algorithm consists of the following steps:
    /// 1. Sample a point from the sampling distribution.
    /// 2. Find the nearest node in the tree to the sample point.
    /// 3. Steer the nearest node towards the sample point.
    /// 4. Add the new node to as a child of the nearest node if the edge is valid.
    /// 5. If the goal is reached, update the solution node.
    fn iteration(&mut self) {
        // Sample a point from the sampling distribution.
        let sample = self.sampling_distribution.sample();

        // Find the nearest node in the tree to the sample point.
        let nearest_node_index = self.nearest_neighbors.nearest_one(&sample).unwrap();
        let nearest_point = &self.nodes[nearest_node_index].point;

        // Steer the nearest node towards the sample point to get a new point.
        let new_point = self.steering.steer(nearest_point, &sample);

        // If the new point or edge is invalid, return.
        if !self.validity_checker.is_point_valid(&new_point)
            || !self
                .validity_checker
                .is_edge_valid(nearest_point, &new_point)
        {
            return;
        }

        // Add the new node to as a child of the nearest node.
        let new_node = Node::new(new_point, Some(nearest_node_index));
        let new_node_index = self.add_node(new_node);

        // If the goal is reached, update the solution node.
        let dist_squared = euclidean_distance_squared(&new_point, &self.goal);
        if dist_squared <= self.goal_tolerance * self.goal_tolerance {
            self.solution = Some(new_node_index);
        }
    }

    /// Adds a node to the tree and the nearest neighbors data structure.
    fn add_node(&mut self, node: Node<F, N>) -> usize {
        let index = self.nodes.len();
        self.nearest_neighbors.add(node.point().clone(), index);
        self.nodes.push(node);
        index
    }
}
