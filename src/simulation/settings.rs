use std::fs::File;
use std::io;
use std::io::BufWriter;
use std::io::Write;

pub struct Settings {
    pub total_time: usize,
    pub nr_robots: usize,
    pub nr_requests: u64,
    pub output_file: Option<String>,
}

impl Settings {
    pub fn write(&self, writer: &mut BufWriter<File>) -> io::Result<()> {
        writer.write("# Number of robots\n".as_bytes())?;
        writer.write(format!("{}\n", self.nr_robots).as_bytes())?;
        writer.write("###\n".as_bytes())?;

        writer.flush()
    }
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            total_time: 15,
            nr_robots: 2,
            nr_requests: 4,
            output_file: None,
        }
    }
}
