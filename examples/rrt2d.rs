//! # Rapidly-exploring Random Tree (RRT) Example in 2 Dimensions
//!
//! ## Usage
//! Run the program with:
//! ```bash
//! cargo run --example rrt2d -- --save-path <save_path> --solution-frames <solution_frames>
//! ```

use clap::Parser;
use image::{ImageBuffer, Rgba};
use macroquad::prelude::*;

const SCREEN_HEIGHT: i32 = 600;
const SCREEN_WIDTH: i32 = 600;

#[derive(Parser)]
struct Args {
    /// The path to save the images to
    #[arg(long, default_value = "frames")]
    save_path: String,

    /// The number of frames to save for the solution
    #[arg(long, default_value_t = 100)]
    solution_frames: usize,
}

async fn save_frame(save_path: &str, frame_number: usize) {
    let screen_image = macroquad::texture::get_screen_data();
    let width = screen_image.width as u32;
    let height = screen_image.height as u32;

    // Convert Macroquad's image to an ImageBuffer from the `image` crate
    let buffer = ImageBuffer::<Rgba<u8>, _>::from_raw(
        width,
        height,
        screen_image.bytes.to_vec(), // Raw pixel data
    )
    .expect("Failed to create image buffer");

    // Save the buffer to a PNG file
    buffer
        .save(format!("{}/frame_{:03}.png", save_path, frame_number))
        .expect("Failed to save image");
}

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
    let args = Args::parse();

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

    let mut frame_count = 0;
    let mut n_nodes = 0;

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

        // Save the frame if the number of nodes has changed.
        if n_nodes != nodes.len() {
            frame_count += 1;
            n_nodes = nodes.len();
            save_frame(&args.save_path, frame_count).await;

            // If the solution was found, make extra frames to show the solution.
            if rrt.solved() {
                for _ in 0..args.solution_frames {
                    frame_count += 1;
                    save_frame(&args.save_path, frame_count).await;
                }
            }
        }

        next_frame().await;
    }
}
