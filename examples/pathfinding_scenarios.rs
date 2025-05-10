use reeds_shepp_lib::{Pose, get_optimal_path, path_length};

fn process_path_request(id: &str, start: Pose, end: Pose) {
    println!("\n--- Path Request: {} ---", id);
    println!("Attempting to find sequence from {:?} to {:?}", start, end);
    match get_optimal_path(start, end) {
        Some(path) => {
            if path.is_empty() {
                println!("Result: Sequence found, but it's empty. Length: 0.00");
            } else {
                println!(
                    "Result: Sequence found. Length: {:.2}. Segments: {}",
                    path_length(&path),
                    path.len()
                );
                for (i, segment) in path.iter().enumerate() {
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
            println!("Result: No sequence found.");
        }
    }
}

fn main() {
    // Case 1: A typical sequence
    let start1 = Pose {
        x: 0.0,
        y: 0.0,
        theta_degree: 0.0,
    };
    let end1 = Pose {
        x: 7.0,
        y: -2.0,
        theta_degree: -45.0,
    };
    process_path_request("Typical", start1, end1);

    // Case 2: Start and End poses are identical
    let start2 = Pose {
        x: 1.0,
        y: 1.0,
        theta_degree: 90.0,
    };
    let end2 = Pose {
        x: 1.0,
        y: 1.0,
        theta_degree: 90.0,
    };
    process_path_request("Identical Poses", start2, end2);

    // Case 3: Start and End poses are very close
    let start3 = Pose {
        x: 0.0,
        y: 0.0,
        theta_degree: 0.0,
    };
    let end3 = Pose {
        x: 0.001,
        y: 0.001,
        theta_degree: 1.0,
    };
    process_path_request("Very Close Poses", start3, end3);

    // Case 4: Poses that might require more complex maneuvers or test edge conditions
    let start4 = Pose {
        x: 0.0,
        y: 0.0,
        theta_degree: 0.0,
    };
    let end4 = Pose {
        x: 0.1,
        y: 0.1,
        theta_degree: 170.0,
    }; // Short distance, large turn
    process_path_request("Challenging Short Turn", start4, end4);

    let start5 = Pose {
        x: 0.0,
        y: 0.0,
        theta_degree: 0.0,
    };
    let end5 = Pose {
        x: 100.0,
        y: 0.0,
        theta_degree: 0.0,
    }; // Long straight sequence
    process_path_request("Long Straight Sequence", start5, end5);
}
