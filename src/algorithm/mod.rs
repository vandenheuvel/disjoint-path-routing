use fnv::FnvHashMap;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::settings::Settings;
use simulation::state::History;
use simulation::Instructions;

pub mod greedy_shortest_paths;
pub mod time_graph;

pub trait Algorithm<'p, 's> {
    fn instantiate(plan: &'p impl Plan, settings: &'s Settings) -> Self
    where
        Self: Sized;
    fn initialize(&mut self, requests: &FnvHashMap<usize, Request>) -> Result<(), NoSolutionError>;
    fn next_step(&mut self, history: &History) -> Instructions;
}

pub struct NoSolutionError {
    message: String,
}

impl NoSolutionError {
    fn new(message: String) -> NoSolutionError {
        NoSolutionError { message }
    }
}
