pub struct Settings {
    pub total_time: usize,
    pub maximum_robots: usize,
    pub nr_requests: u64,
    pub real_time: bool,
    pub output_file: Option<String>,
    // pub failure_rate: f64,
    // pub taxi_or_paths: TaxiOrPath,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            total_time: 15,
            maximum_robots: 2,
            nr_requests: 4,
            real_time: false,
            output_file: None,
        }
    }
}
