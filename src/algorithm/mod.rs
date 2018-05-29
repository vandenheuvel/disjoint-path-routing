use simulation::simulation::Instructions;
use simulation::plan::Plan;
use simulation::settings::Settings;
use simulation::simulation::State;

pub mod greedy_shortest_paths;
pub mod time_graph;

pub trait Algorithm<'p, 's> {
    fn instantiate(plan: &'p Plan, settings: &'s Settings) -> Self where Self: Sized;
    fn next_step(&mut self, history: &Vec<State>) -> Instructions;
}
