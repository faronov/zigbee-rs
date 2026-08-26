//! Compatibility alias for the shared finite SensorApp host demo.

fn main() {
    println!("mock-sensor now aliases mock-sleepy-sensor");
    pollster::block_on(mock_sleepy_sensor::run_demo());
}
