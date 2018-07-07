use algorithm::assignment::AssignmentAlgorithm;
use fnv::FnvHashMap;
use priority_queue::PriorityQueue;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::plan::Vertex;
use simulation::settings::Settings;
use std::cmp::Reverse;
use std::iter::repeat;

pub struct GreedyMakespan<'p, 's> {
    plan: &'p dyn Plan,
    settings: &'s Settings,
}

impl<'p, 's> GreedyMakespan<'p, 's> {
    pub fn new(plan: &'p dyn Plan, settings: &'s Settings) -> GreedyMakespan<'p, 's> {
        GreedyMakespan { plan, settings }
    }
}

impl<'p, 's> AssignmentAlgorithm<'p, 's> for GreedyMakespan<'p, 's> {
    fn calculate_assignment(
        &mut self,
        requests: &FnvHashMap<usize, Request>,
        availability: &Vec<(usize, Vertex)>,
    ) -> Vec<Vec<usize>> {
        let mut availability = availability
            .iter()
            .enumerate()
            .map(|(robot, time)| (robot, Reverse(time.0)))
            .collect::<PriorityQueue<_, _>>();

        let mut assigned_paths = repeat(Vec::new())
            .take(self.settings.nr_robots)
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

#[cfg(test)]
mod test {

    use fnv::FnvHashMap;

    use algorithm::assignment::greedy_makespan::GreedyMakespan;
    use algorithm::assignment::test::get_test_variables;
    use algorithm::assignment::AssignmentAlgorithm;
    use simulation::demand::Request;
    use simulation::plan::Vertex;

    #[test]
    fn calculate_assignment_single() {
        let (plan, settings) = get_test_variables(1, 1);
        let mut algorithm = GreedyMakespan::new(&plan, &settings);

        let requests = map![0 => Request {
            from: Vertex { x: 0, y: 0, },
            to: Vertex { x: 0, y: 1, },
        },];
        let availability = vec![(0, Vertex { x: 0, y: 1 })];

        assert_eq!(
            vec![vec![0]],
            algorithm.calculate_assignment(&requests, &availability)
        );
    }

    #[test]
    fn calculate_assignment_multiple() {
        let (plan, settings) = get_test_variables(2, 3);
        let mut algorithm = GreedyMakespan::new(&plan, &settings);

        let requests = map![
            0 => Request {
                from: Vertex { x: 0, y: 0, },
                to: Vertex { x: 0, y: 1, },
            },
            1 => Request {
                from: Vertex { x: 0, y: 0, },
                to: Vertex { x: 0, y: 2, },
            },
            2 => Request {
                from: Vertex { x: 0, y: 0, },
                to: Vertex { x: 0, y: 1, },
            },
        ];

        let availability = vec![(0, Vertex { x: 0, y: 0 }), (1, Vertex { x: 0, y: 0 })];

        let assignment = algorithm.calculate_assignment(&requests, &availability);
        assert_eq!(assignment.len(), 2);
        let totals: Vec<u64> = assignment
            .into_iter()
            .map(|assigned| {
                assigned
                    .into_iter()
                    .map(|id| requests.get(&id).unwrap())
                    .map(Request::distance)
            })
            .map(Iterator::sum)
            .collect();
        assert_eq!(totals, vec![2, 2]);
    }
}
