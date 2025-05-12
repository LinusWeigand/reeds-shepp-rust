use reeds_shepp_lib::{Pose, get_all_paths, path_length};

fn main() {
    let start_pose = Pose {
        x: 0.0,
        y: 0.0,
        theta_degree: 0.0,
    };
    let end_pose = Pose {
        x: 2.0,
        y: 2.0,
        theta_degree: 90.0,
    };

    println!("Calculating all candidate movement sequences (before optimization)...");
    println!("Start: {:?}", start_pose);
    println!("End:   {:?}", end_pose);

    let all_paths = get_all_paths(start_pose, end_pose);

    if all_paths.is_empty() {
        println!("\nNo candidate sequences were generated.");
    } else {
        println!("\nFound {} candidate sequences:", all_paths.len());
        for (i, path) in all_paths.iter().enumerate() {
            if path.is_empty() {
                println!(
                    "  Sequence {}: Empty (filtered out due to zero-length segments)",
                    i + 1
                );
            } else {
                println!(
                    "  Sequence {}: Length: {:.2}, Segments: {}",
                    i + 1,
                    path_length(path),
                    path.len()
                );
                // Optionally print segment details for each sequence
                // for (j, segment) in path.iter().enumerate() {
                //     println!(
                //         "    Segment {}: Param: {:.2}, Steering: {:?}, Gear: {:?}",
                //         j + 1,
                //         segment.param,
                //         segment.steering,
                //         segment.gear
                //     );
                // }
            }
        }

        let optimal_from_all = all_paths
            .into_iter()
            .filter(|p| !p.is_empty())
            .min_by(|a, b| path_length(a).partial_cmp(&path_length(b)).unwrap());

        match optimal_from_all {
            Some(optimal_path) => {
                println!("\nManually selected optimal sequence from the list:");
                println!("Length: {:.2}", path_length(&optimal_path));
                for (k, segment) in optimal_path.iter().enumerate() {
                    println!(
                        "  Segment {}: Param: {:.2}, Steering: {:?}, Gear: {:?}",
                        k + 1,
                        segment.param,
                        segment.steering,
                        segment.gear
                    );
                }
            }
            None => {
                println!(
                    "\nCould not select an optimal sequence from the generated list (all might have been empty)."
                );
            }
        }
    }
}
