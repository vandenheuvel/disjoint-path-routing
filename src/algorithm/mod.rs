pub mod assignment;
pub mod path;

pub struct NoSolutionError {
    message: String,
}

impl NoSolutionError {
    fn new(message: String) -> NoSolutionError {
        NoSolutionError { message }
    }
}
