use std::f64::consts::PI;

pub mod utils;
pub use utils::Pose;
pub use utils::normalize_angle_rad;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Steering {
    Left,
    Right,
    Straight,
}

impl Steering {
    fn reverse(&self) -> Self {
        match self {
            Steering::Left => Steering::Right,
            Steering::Right => Steering::Left,
            Steering::Straight => Steering::Straight,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gear {
    Forward,
    Backwards,
}

impl Gear {
    fn reverse(&self) -> Self {
        match self {
            Gear::Forward => Gear::Backwards,
            Gear::Backwards => Gear::Forward,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PathElement {
    pub param: f64,
    pub steering: Steering,
    pub gear: Gear,
}

impl PathElement {
    fn create(param: f64, steering: Steering, gear: Gear) -> Self {
        return if param >= 0. {
            PathElement {
                param,
                steering,
                gear,
            }
        } else {
            PathElement {
                param: -param,
                steering,
                gear: gear.reverse(),
            }
        };
    }

    fn reverse_steering(&mut self) {
        self.steering = self.steering.reverse();
    }

    fn reverse_gear(&mut self) {
        self.gear = self.gear.reverse();
    }
}

pub type Path = Vec<PathElement>;

pub fn get_optimal_path(start: Pose, end: Pose) -> Option<Path> {
    let paths = get_all_paths(start, end);

    paths
        .into_iter()
        .min_by(|a, b| path_length(a).partial_cmp(&path_length(b)).unwrap())
}

pub type PathFn = fn(f64, f64, f64) -> Path;
pub const PATH_FNS: [PathFn; 12] = [
    path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12,
];

pub fn get_all_paths(start: Pose, end: Pose) -> Vec<Path> {
    let mut paths: Vec<Path> = Vec::new();

    let pose = utils::change_of_basis(&start, &end);
    let x = pose.x;
    let y = pose.y;
    let theta_degree = pose.theta_degree;
    for get_path in PATH_FNS {
        let p1 = get_path(x, y, theta_degree);
        let p2 = timeflip(get_path(-x, y, -theta_degree));
        let p3 = reflect(get_path(x, -y, -theta_degree));
        let p4 = reflect(timeflip(get_path(-x, -y, theta_degree)));

        paths.push(p1);
        paths.push(p2);
        paths.push(p3);
        paths.push(p4);
    }

    paths
        .into_iter()
        .map(|path| {
            path.into_iter()
                .filter(|e| e.param.abs() > 1e-10)
                .collect::<Path>()
        })
        .filter(|path| !path.is_empty())
        .collect()
}

pub fn timeflip(path: Path) -> Path {
    path.into_iter()
        .map(|mut e| {
            e.reverse_gear();
            e
        })
        .collect()
}

pub fn reflect(path: Path) -> Path {
    path.into_iter()
        .map(|mut e| {
            e.reverse_steering();
            e
        })
        .collect()
}

pub fn path_length(path: &Path) -> f64 {
    path.iter().map(|e| e.param.abs()).sum()
}

fn path1(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);
    let polar = utils::cartesian_to_polar(x - phi_radians.sin(), y - 1. + phi_radians.cos());
    let v = utils::normalize_angle_rad(phi_radians - polar.theta);

    vec![
        PathElement::create(polar.theta, Steering::Left, Gear::Forward),
        PathElement::create(polar.rho, Steering::Straight, Gear::Forward),
        PathElement::create(v, Steering::Left, Gear::Forward),
    ]
}

fn path2(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::normalize_angle_rad(utils::degree_to_radians(phi_degree));
    let polar = utils::cartesian_to_polar(x + phi_radians.sin(), y - 1. - phi_radians.cos());

    let rho = polar.rho;
    let theta = polar.theta;

    if rho * rho >= 4. {
        let u = (rho * rho - 4.).sqrt();
        let t = utils::normalize_angle_rad(theta + (2.0_f64).atan2(u));
        let v = utils::normalize_angle_rad(t - phi_radians);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(u, Steering::Straight, Gear::Forward),
            PathElement::create(v, Steering::Right, Gear::Forward),
        ]
    } else {
        Vec::new()
    }
}

fn path3(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x - phi_radians.sin();
    let eta = y - 1. + phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if polar.rho <= 4. {
        let a = (rho / 4.).acos();
        let t = utils::normalize_angle_rad(theta + PI / 2. + a);
        let u = utils::normalize_angle_rad(PI - 2. * a);
        let v = utils::normalize_angle_rad(phi_radians - t - u);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(u, Steering::Right, Gear::Backwards),
            PathElement::create(v, Steering::Left, Gear::Forward),
        ]
    } else {
        Vec::new()
    }
}

fn path4(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x - phi_radians.sin();
    let eta = y - 1. + phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho <= 4. {
        let a = (rho / 4.).acos();
        let t = utils::normalize_angle_rad(theta + PI / 2. + a);
        let u = utils::normalize_angle_rad(PI - 2. * a);
        let v = utils::normalize_angle_rad(t + u - phi_radians);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(u, Steering::Right, Gear::Backwards),
            PathElement::create(v, Steering::Left, Gear::Backwards),
        ]
    } else {
        Vec::new()
    }
}

fn path5(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x - phi_radians.sin();
    let eta = y - 1. + phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho <= 4. {
        let u = (1. - rho * rho / 8.).acos();
        let a = (2. * u.sin() / rho).asin();
        let t = utils::normalize_angle_rad(theta + PI / 2. - a);
        let v = utils::normalize_angle_rad(t - u - phi_radians);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(u, Steering::Right, Gear::Forward),
            PathElement::create(v, Steering::Left, Gear::Backwards),
        ]
    } else {
        Vec::new()
    }
}

fn path6(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x + phi_radians.sin();
    let eta = y - 1. - phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho <= 4. {
        let (t, u, v);
        if rho <= 2. {
            let a = ((rho + 2.) / 4.).acos();
            t = utils::normalize_angle_rad(theta + PI / 2. + a);
            u = utils::normalize_angle_rad(a);
            v = utils::normalize_angle_rad(phi_radians - t + 2. * u);
        } else {
            let a = ((rho - 2.) / 4.).acos();
            t = utils::normalize_angle_rad(theta + PI / 2. - a);
            u = utils::normalize_angle_rad(PI - a);
            v = utils::normalize_angle_rad(phi_radians - t + 2. * u);
        }

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(u, Steering::Right, Gear::Forward),
            PathElement::create(u, Steering::Left, Gear::Backwards),
            PathElement::create(v, Steering::Right, Gear::Backwards),
        ]
    } else {
        Vec::new()
    }
}

fn path7(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x + phi_radians.sin();
    let eta = y - 1. - phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    let u1 = (20. - rho * rho) / 16.;

    if rho <= 6. && u1 >= 0. && u1 <= 1. {
        let u = u1.acos();
        let asin_arg = (2. * u.sin() / rho).min(1.0).max(-1.0);
        let a = asin_arg.asin();
        let t = utils::normalize_angle_rad(theta + PI / 2. + a);
        let v = utils::normalize_angle_rad(t - phi_radians);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(u, Steering::Right, Gear::Backwards),
            PathElement::create(u, Steering::Left, Gear::Backwards),
            PathElement::create(v, Steering::Right, Gear::Forward),
        ]
    } else {
        Vec::new()
    }
}

fn path8(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x - phi_radians.sin();
    let eta = y - 1. + phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 2. {
        let sqrt_arg = rho * rho - 4.;
        if sqrt_arg < 0. {
            return Vec::new();
        }

        let s = sqrt_arg.sqrt();
        let u_param = s - 2.;

        let a = (2.0_f64).atan2(s);
        let t = utils::normalize_angle_rad(theta + PI / 2. + a);
        let v = utils::normalize_angle_rad(t - phi_radians + PI / 2.);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(PI / 2., Steering::Right, Gear::Backwards),
            PathElement::create(u_param, Steering::Straight, Gear::Backwards),
            PathElement::create(v, Steering::Left, Gear::Backwards),
        ]
    } else {
        Vec::new()
    }
}

fn path9(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x - phi_radians.sin();
    let eta = y - 1. + phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 2. {
        let sqrt_arg = rho * rho - 4.;
        if sqrt_arg < 0. {
            return Vec::new();
        }

        let s = sqrt_arg.sqrt();
        let u_param = s - 2.;

        let a = s.atan2(2.0_f64);
        let t = utils::normalize_angle_rad(theta + PI / 2. - a);
        let v = utils::normalize_angle_rad(t - phi_radians - PI / 2.);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(u_param, Steering::Straight, Gear::Forward),
            PathElement::create(PI / 2., Steering::Right, Gear::Forward),
            PathElement::create(v, Steering::Left, Gear::Backwards),
        ]
    } else {
        Vec::new()
    }
}

fn path10(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x + phi_radians.sin();
    let eta = y - 1. - phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 2. {
        let t = utils::normalize_angle_rad(theta + PI / 2.);
        let u = rho - 2.;
        let v = utils::normalize_angle_rad(phi_radians - t - PI / 2.);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(PI / 2., Steering::Right, Gear::Backwards),
            PathElement::create(u, Steering::Straight, Gear::Backwards),
            PathElement::create(v, Steering::Right, Gear::Backwards),
        ]
    } else {
        Vec::new()
    }
}

fn path11(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x + phi_radians.sin();
    let eta = y - 1. - phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 2. {
        let t = utils::normalize_angle_rad(theta);
        let u = rho - 2.;
        let v = utils::normalize_angle_rad(phi_radians - t - PI / 2.);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(u, Steering::Straight, Gear::Forward),
            PathElement::create(PI / 2., Steering::Left, Gear::Forward),
            PathElement::create(v, Steering::Right, Gear::Backwards),
        ]
    } else {
        Vec::new()
    }
}

fn path12(x: f64, y: f64, phi_degree: f64) -> Path {
    let phi_radians = utils::degree_to_radians(phi_degree);

    let xi = x + phi_radians.sin();
    let eta = y - 1. - phi_radians.cos();
    let polar = utils::cartesian_to_polar(xi, eta);

    let rho = polar.rho;
    let theta = polar.theta;

    if rho >= 4. {
        let sqrt_base_arg = rho * rho - 4.;
        if sqrt_base_arg < 0. {
            return Vec::new();
        }

        let u_base = sqrt_base_arg.sqrt();
        let u_param = u_base - 4.;

        let s_equiv = u_base;

        let a = (2.0_f64).atan2(s_equiv);
        let t = utils::normalize_angle_rad(theta + PI / 2. + a);
        let v = utils::normalize_angle_rad(t - phi_radians);

        vec![
            PathElement::create(t, Steering::Left, Gear::Forward),
            PathElement::create(PI / 2., Steering::Right, Gear::Backwards),
            PathElement::create(u_param, Steering::Straight, Gear::Backwards),
            PathElement::create(PI / 2., Steering::Left, Gear::Backwards),
            PathElement::create(v, Steering::Right, Gear::Forward),
        ]
    } else {
        Vec::new()
    }
}
