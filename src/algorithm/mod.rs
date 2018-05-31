use simulation::simulation::Instructions;
use simulation::plan::Plan;
use simulation::settings::Settings;
use std::collections::HashMap;
use simulation::demand::Request;
use simulation::simulation::History;

pub mod greedy_shortest_paths;
pub mod time_graph;

pub trait Algorithm<'p, 's> {
    fn instantiate(plan: &'p impl Plan, settings: &'s Settings) -> Self where Self: Sized;
    fn initialize(&mut self, requests: &HashMap<usize, Request>);
    fn next_step(&mut self, history: &History) -> Instructions;
}
