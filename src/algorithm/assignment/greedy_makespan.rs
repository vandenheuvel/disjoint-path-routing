use algorithm::assignment::AssignmentAlgorithm;
use fnv::FnvHashMap;
use priority_queue::PriorityQueue;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::settings::Settings;
use std::cmp::Reverse;
use std::iter::repeat;

pub struct GreedyMakespan<'p, 's> {
    plan: &'p (Plan + 'p),
    settings: &'s Settings,
}

impl<'p, 's> AssignmentAlgorithm<'p, 's> for GreedyMakespan<'p, 's> {
    fn instantiate(plan: &'p impl Plan, settings: &'s Settings) -> Self
    where
        Self: Sized,
    {
        GreedyMakespan { plan, settings }
    }
    fn calculate_assignment(
        &mut self,
        requests: &FnvHashMap<usize, Request>,
        availability: Vec<usize>,
    ) -> Vec<Vec<usize>> {
        let mut availability = availability
            .into_iter()
            .enumerate()
            .map(|(robot, time)| (robot, Reverse(time)))
            .collect::<PriorityQueue<_, _>>();

        let mut assigned_paths = repeat(Vec::new())
            .take(self.settings.maximum_robots)
            .collect::<Vec<_>>();

        for (&request, &Request { from, to }) in requests.iter() {
            let (&robot, &Reverse(time_available)) = availability.peek().unwrap();

            assigned_paths[robot].push(request);
            availability.change_priority(
                &robot,
                Reverse(time_available + 1 + self.plan.path_length(from, to) as usize + 1),
            );
        }

        assigned_paths
    }
}
