use reeds_shepp_lib::{Pose, get_optimal_path, path_length};

fn main() {
    let start_pose = Pose {
        x: 0.0,
        y: 0.0,
        theta_degree: 0.0,
    };
    let end_pose = Pose {
        x: 5.0,
        y: 3.0,
        theta_degree: 90.0,
    };

    println!("Calculating optimal movement sequence...");
    println!(
        "Start: x={:.2}, y={:.2}, theta={:.2}°",
        start_pose.x, start_pose.y, start_pose.theta_degree
    );
    println!(
        "End:   x={:.2}, y={:.2}, theta={:.2}°",
        end_pose.x, end_pose.y, end_pose.theta_degree
    );

    match get_optimal_path(start_pose, end_pose) {
        Some(optimal_path) => {
            if optimal_path.is_empty() {
                println!(
                    "\nAn optimal sequence was found, but it's empty (start and end might be too close or identical after filtering)."
                );
                println!("Sequence length: 0.00");
            } else {
                println!("\nOptimal movement sequence found!");
                println!("Total sequence length: {:.2}", path_length(&optimal_path));
                println!("Sequence segments:");
                for (i, segment) in optimal_path.iter().enumerate() {
                    println!(
                        "  Segment {}: Param: {:.2}, Steering: {:?}, Gear: {:?}",
                        i + 1,
                        segment.param,
                        segment.steering,
                        segment.gear
                    );
                }
            }
        }
        None => {
            println!("\nNo optimal movement sequence could be found between the specified poses.");
        }
    }

    println!("\n--- Another example ---");
    let start_pose_2 = Pose {
        x: 1.0,
        y: 1.0,
        theta_degree: 45.0,
    };
    let end_pose_2 = Pose {
        x: -2.0,
        y: 4.0,
        theta_degree: -30.0,
    };
    println!(
        "Start: x={:.2}, y={:.2}, theta={:.2}°",
        start_pose_2.x, start_pose_2.y, start_pose_2.theta_degree
    );
    println!(
        "End:   x={:.2}, y={:.2}, theta={:.2}°",
        end_pose_2.x, end_pose_2.y, end_pose_2.theta_degree
    );

    if let Some(path) = get_optimal_path(start_pose_2, end_pose_2) {
        println!("\nOptimal movement sequence found!");
        println!("Total sequence length: {:.2}", path_length(&path));
        for (i, segment) in path.iter().enumerate() {
            println!(
                "  Segment {}: Param: {:.2}, Steering: {:?}, Gear: {:?}",
                i + 1,
                segment.param,
                segment.steering,
                segment.gear
            );
        }
    } else {
        println!("\nNo optimal movement sequence could be found for the second example.");
    }
}
