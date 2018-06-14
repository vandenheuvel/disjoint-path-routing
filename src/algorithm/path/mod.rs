use algorithm::assignment::AssignmentAlgorithm;
use algorithm::NoSolutionError;
use fnv::FnvHashSet;
use simulation::plan::Plan;
use simulation::settings::Settings;
use simulation::state::History;
use simulation::Instructions;

pub mod greedy_shortest_paths;
mod time_graph;

pub trait PathAlgorithm<'p, 's, 'a> {
    fn instantiate(
        plan: &'p impl Plan,
        settings: &'s Settings,
        assignment_algorithm: Box<impl AssignmentAlgorithm<'p, 's> + 'a>,
    ) -> Self
    where
        Self: Sized;
    fn initialize(&mut self) -> Result<(), NoSolutionError>;
    fn next_step(&mut self, history: &History) -> Instructions;
    fn contains_new_requests(&self, history: &History) -> bool {
        history.time() == 1 || {
            let new_requests = history
                .last_state()
                .requests
                .keys()
                .map(|request| *request)
                .collect::<FnvHashSet<_>>();
            let old_requests = history.states[history.time() - 2]
                .requests
                .keys()
                .map(|request| *request)
                .collect::<FnvHashSet<_>>();

            new_requests.difference(&old_requests).count() > 0
        }
    }
}
