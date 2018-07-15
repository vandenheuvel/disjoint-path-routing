pub mod assignment;
pub mod path;

const RUN_FILE_NAME: &str = "run.run";
const DAT_FILE_NAME: &str = "data.dat";

pub struct NoSolutionError {
    message: String,
}

impl NoSolutionError {
    fn new(message: String) -> NoSolutionError {
        NoSolutionError { message }
    }
}
