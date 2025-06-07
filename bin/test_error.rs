use auto_aim_rust::rbt_err::RbtError;

fn main() {
    println!("Hello, world!");
    let err1 = RbtError::NoCamera;
    println!("{}", err1);
}
