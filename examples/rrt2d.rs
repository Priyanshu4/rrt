//! # Rapidly-exploring Random Tree (RRT) Example in 2 Dimensions
//!
//! ## Usage
//! Run the program with:
//! ```bash
//! cargo run --example rrt2d
//! ```

use macroquad::prelude::*;

const SCREEN_HEIGHT: i32 = 600;
const SCREEN_WIDTH: i32 = 600;

fn window_conf() -> Conf {
    Conf {
        window_title: "RRT in a 2D Environment with Spherical Obstacles".to_string(),
        window_width: SCREEN_HEIGHT,
        window_height: SCREEN_WIDTH,
        window_resizable: false,
        fullscreen: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    // Define the obstacles
    let spheres = vec![
        rrt::collision::Sphere {
            center: rrt::Point::new([400.0, 400.0]),
            radius: 50.0,
        },
        rrt::collision::Sphere {
            center: rrt::Point::new([400.0, 320.0]),
            radius: 50.0,
        },
        rrt::collision::Sphere {
            center: rrt::Point::new([200.0, 200.0]),
            radius: 100.0,
        },
        rrt::collision::Sphere {
            center: rrt::Point::new([300.0, 200.0]),
            radius: 100.0,
        },
        rrt::collision::Sphere {
            center: rrt::Point::new([400.0, 200.0]),
            radius: 100.0,
        },
        rrt::collision::Sphere {
            center: rrt::Point::new([200.0, 420.0]),
            radius: 100.0,
        },
    ];
    // We clone the spheres so that we have still have an unmoved copy that we can use for drawing.
    let validity_checker = rrt::collision::EuclideanSphericalObstacleSet::new(spheres.clone());

    // Define the start and goal points.
    let start = rrt::Point::new([100.0, 100.0]);
    let goal = rrt::Point::new([500.0, 500.0]);
    let goal_tolerance = 5.0;

    // Define the steering function.
    let steering = rrt::steering::EuclideanSteering::new(20.0);

    // Use a uniform sampling distribution with 5% goal bias.
    let ranges = [(0.0, SCREEN_WIDTH as f32), (0.0, SCREEN_HEIGHT as f32)];
    let goal_bias = 0.05;
    let result = rrt::sampling::GoalBiasedUniformDistribution::new(ranges, goal, goal_bias);
    if result.is_err() {
        println!(
            "Error creating sampling distribution: {}",
            result.err().unwrap()
        );
        return;
    }
    let sampling_distribution = result.unwrap();

    // Create the RRT planner.
    let mut rrt = rrt::RRT::<f32, 2, _, _, _, rrt::KdTreeNearestNeighbors<_, 2>>::new(
        start,
        goal,
        goal_tolerance,
        validity_checker,
        sampling_distribution,
        steering,
    );

    loop {
        // Clear the screen
        clear_background(WHITE);

        // Draw the obstacles
        for sphere in &spheres {
            draw_circle(sphere.center[0], sphere.center[1], sphere.radius, BLACK);
        }

        // Draw the start and goal points.
        draw_circle(start[0], start[1], 5.0, BLUE);
        draw_circle(goal[0], goal[1], goal_tolerance, GREEN);

        if !rrt.solved() {
            rrt.run_iterations(1);
        }

        // Draw each node and the edge to its parent.
        let nodes: &Vec<rrt::rrt::Node<f32, 2>> = rrt.get_tree();
        for node in nodes {
            let point = node.point();
            if let Some(parent_index) = node.parent() {
                let parent = &nodes[parent_index];
                let parent_point = parent.point();
                draw_line(
                    point[0],
                    point[1],
                    parent_point[0],
                    parent_point[1],
                    1.0,
                    BLACK,
                );
            }
            draw_circle(point[0], point[1], 2.0, BLACK);
        }

        // Draw the path if a solution was found.
        if let Some(path) = rrt.get_path() {
            // Raw path
            for i in 0..path.len() - 1 {
                let a = path[i];
                let b = path[i + 1];
                draw_line(a[0], a[1], b[0], b[1], 2.0, RED);
            }

            // Smooth the path using shortcutting
            let shortened_path =
                rrt::smoothing::fast_shortcutting(path, rrt.get_validity_checker());
            for i in 0..shortened_path.len() - 1 {
                let a = shortened_path[i];
                let b = shortened_path[i + 1];
                draw_line(a[0], a[1], b[0], b[1], 2.0, GREEN);
            }
        }

        next_frame().await;
    }
}
