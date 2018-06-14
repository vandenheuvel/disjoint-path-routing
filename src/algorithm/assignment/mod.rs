use fnv::FnvHashMap;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::settings::Settings;

pub mod greedy_makespan;
pub mod pdp;

pub trait AssignmentAlgorithm<'p, 's> {
    fn instantiate(plan: &'p impl Plan, settings: &'s Settings) -> Self
    where
        Self: Sized;
    fn calculate_assignment(
        &mut self,
        requests: &FnvHashMap<usize, Request>,
        availabilities: Vec<usize>,
    ) -> Vec<Vec<usize>>;
}
