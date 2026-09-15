fn main() {
    pollster::block_on(mock_sleepy_sensor::run_demo());
}
