use simulation::settings::AssignmentMethod::Single;
use std::fs::File;
use std::io;
use std::io::BufWriter;
use std::io::Write;

pub struct Settings {
    pub total_time: usize,
    pub maximum_robots: usize,
    pub nr_requests: u64,
    pub real_time: bool,
    pub output_file: Option<String>,
    pub assignment: AssignmentMethod,
    // pub failure_rate: f64,
    // pub taxi_or_paths: TaxiOrPath,
}

impl Settings {
    pub fn write(&self, writer: &mut BufWriter<File>) -> io::Result<()> {
        writer.write("# Number of robots\n".as_bytes())?;
        writer.write(format!("{}\n", self.maximum_robots).as_bytes())?;
        writer.write("###\n".as_bytes())?;

        writer.flush()
    }
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            total_time: 15,
            maximum_robots: 2,
            nr_requests: 4,
            real_time: false,
            output_file: None,
            assignment: Single,
        }
    }
}

pub enum AssignmentMethod {
    Single,
    Multiple,
}
