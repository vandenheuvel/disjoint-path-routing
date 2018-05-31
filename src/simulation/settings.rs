pub struct Settings {
    pub total_time: usize,
    pub maximum_robots: usize,
    pub nr_requests: u64,
    // pub real_time: bool,
    // pub failure_rate: f64,
    // pub taxi_or_paths: TaxiOrPath,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            total_time: 10,
            maximum_robots: 1,
            nr_requests: 10,
        }
    }
}
