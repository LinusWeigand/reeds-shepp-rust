use std::f64::consts::PI;

pub fn normalize_angle_rad(theta: f64) -> f64 {
    let mut theta = theta.rem_euclid(2. * PI);
    if theta >= PI {
        theta -= 2. * PI
    } else if theta < -PI {
        theta += 2. * PI
    }
    theta
}

pub struct Polar {
    pub rho: f64,
    pub theta: f64,
}

pub fn cartesian_to_polar(x: f64, y: f64) -> Polar {
    let rho = x.hypot(y);
    let theta = y.atan2(x);
    Polar { rho, theta }
}

pub fn degree_to_radians(degree: f64) -> f64 {
    degree * PI / 180.
}

#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta_degree: f64,
}

pub fn change_of_basis(p1: &Pose, p2: &Pose) -> Pose {
    let theta1_radians = degree_to_radians(p1.theta_degree);

    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;

    let cos_theta1 = theta1_radians.cos();
    let sin_theta1 = theta1_radians.sin();

    let new_x = dx * cos_theta1 + dy * sin_theta1;
    let new_y = -dx * sin_theta1 + dy * cos_theta1;

    let new_theta_degree = p2.theta_degree - p1.theta_degree;

    Pose {
        x: new_x,
        y: new_y,
        theta_degree: new_theta_degree,
    }
}
