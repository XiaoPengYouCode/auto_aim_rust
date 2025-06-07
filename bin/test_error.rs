use auto_aim_rust::robot_error::RobotError;

fn main() {
    println!("Hello, world!");
    let err1 = RobotError::NoCamera;
    println!("{}", err1);
}
