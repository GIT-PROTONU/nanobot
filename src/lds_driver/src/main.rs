//! ROS 2 LDS driver node (Rust / r2r).
//!
//! Reads the Roborock LDS02RR binary stream from a serial port, decodes it with
//! `lds::LdsParser`, and publishes one `sensor_msgs/LaserScan` per revolution on
//! `/scan`. This is the "hot path" of the stack, hence Rust.
//!
//! Parameters (set via --ros-args --params-file robot.yaml):
//!   port (string)      serial device, e.g. /dev/ttyS2
//!   baud (int)         default 115200
//!   frame_id (string)  default "laser"
//!   clockwise (bool)   LDS02RR spins CW; true converts to the ROS CCW frame
//!   angle_offset_deg (double)  mechanical mounting offset of the 0deg ray
//!   range_min/range_max (double)  metres; readings outside become "no return"

mod lds;

use std::f32::consts::PI;
use std::io::Read;
use std::time::Duration;

use r2r::{Context, Node, ParameterValue, QosProfile};

const RAYS: usize = 360;

struct Params {
    port: String,
    baud: u32,
    frame_id: String,
    clockwise: bool,
    angle_offset_deg: f64,
    range_min: f32,
    range_max: f32,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "lds_driver", "")?;
    let p = read_params(&node);

    let publisher =
        node.create_publisher::<r2r::sensor_msgs::msg::LaserScan>("scan", QosProfile::sensor_data())?;
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime)?;

    println!("[lds_driver] opening {} @ {} baud", p.port, p.baud);
    let mut serial = serialport::new(&p.port, p.baud)
        .timeout(Duration::from_millis(200))
        .open()?;

    let mut parser = lds::LdsParser::new();
    let mut rd = [0u8; 1024];

    loop {
        match serial.read(&mut rd) {
            Ok(0) => {}
            Ok(n) => {
                for &b in &rd[..n] {
                    if let Some(scan) = parser.feed(b) {
                        let msg = build_scan(&mut clock, &p, &scan, parser.rpm);
                        if let Err(e) = publisher.publish(&msg) {
                            eprintln!("[lds_driver] publish error: {e}");
                        }
                    }
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => eprintln!("[lds_driver] serial read error: {e}"),
        }
        // Process any parameter/service traffic without blocking the read loop.
        node.spin_once(Duration::from_millis(0));
    }
}

fn build_scan(
    clock: &mut r2r::Clock,
    p: &Params,
    points: &[lds::ScanPoint],
    rpm: f32,
) -> r2r::sensor_msgs::msg::LaserScan {
    let mut ranges = vec![f32::INFINITY; RAYS];
    let mut intensities = vec![0.0f32; RAYS];
    let offset = p.angle_offset_deg.round() as i32;

    for pt in points {
        if pt.dist_mm == 0 {
            continue;
        }
        let dist = pt.dist_mm as f32 / 1000.0;
        if dist < p.range_min || dist > p.range_max {
            continue;
        }
        // sensor reports CW; ROS LaserScan increases CCW. Convert + apply offset.
        let base = if p.clockwise {
            (RAYS as i32 - pt.angle as i32) % RAYS as i32
        } else {
            pt.angle as i32
        };
        let idx = (((base + offset) % RAYS as i32) + RAYS as i32) as usize % RAYS;
        // keep the closest hit if two readings land on the same degree bin
        if dist < ranges[idx] {
            ranges[idx] = dist;
            intensities[idx] = pt.quality as f32;
        }
    }

    let scan_time = if rpm > 1.0 { 60.0 / rpm } else { 0.2 };
    let now = clock.get_now().unwrap_or_default();
    let stamp = r2r::Clock::to_builtin_time(&now);

    r2r::sensor_msgs::msg::LaserScan {
        header: r2r::std_msgs::msg::Header { stamp, frame_id: p.frame_id.clone() },
        angle_min: 0.0,
        angle_max: 2.0 * PI * (RAYS as f32 - 1.0) / RAYS as f32,
        angle_increment: 2.0 * PI / RAYS as f32,
        time_increment: scan_time / RAYS as f32,
        scan_time,
        range_min: p.range_min,
        range_max: p.range_max,
        ranges,
        intensities,
    }
}

/// Pull parameters out of the node (loaded from --params-file), with defaults.
/// NOTE: r2r exposes parameters as `node.params: Arc<Mutex<HashMap<String,
/// ParameterValue>>>`. If you bump r2r and the param API changes, this is the
/// only spot to adjust.
fn read_params(node: &Node) -> Params {
    let map = node.params.lock().unwrap();
    let s = |k: &str, d: &str| match map.get(k) {
        Some(ParameterValue::String(v)) => v.clone(),
        _ => d.to_string(),
    };
    let i = |k: &str, d: i64| match map.get(k) {
        Some(ParameterValue::Integer(v)) => *v,
        _ => d,
    };
    let f = |k: &str, d: f64| match map.get(k) {
        Some(ParameterValue::Double(v)) => *v,
        Some(ParameterValue::Integer(v)) => *v as f64,
        _ => d,
    };
    let b = |k: &str, d: bool| match map.get(k) {
        Some(ParameterValue::Bool(v)) => *v,
        _ => d,
    };
    Params {
        port: s("port", "/dev/ttyS2"),
        baud: i("baud", 115200) as u32,
        frame_id: s("frame_id", "laser"),
        clockwise: b("clockwise", true),
        angle_offset_deg: f("angle_offset_deg", 0.0),
        range_min: f("range_min", 0.12) as f32,
        range_max: f("range_max", 6.0) as f32,
    }
}
