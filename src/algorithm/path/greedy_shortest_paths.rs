use algorithm::assignment::AssignmentAlgorithm;
use algorithm::path::time_graph::TimeGraph;
use algorithm::path::PathAlgorithm;
use algorithm::NoSolutionError;
use fnv::FnvHashMap;
use fnv::FnvHashSet;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::plan::Vertex;
use simulation::settings::Settings;
use simulation::state::History;
use simulation::state::State;
use simulation::Instructions;
use simulation::MoveInstruction;
use simulation::PlacementInstruction;
use simulation::RemovalInstruction;
use std::iter::repeat;

pub struct GreedyShortestPaths<'p, 's, 'a> {
    time_graph: TimeGraph<'p>,
    settings: &'s Settings,
    assignment_algorithm: Box<AssignmentAlgorithm<'p, 's> + 'a>,

    time: usize,
    assignment: Vec<Vec<usize>>,
    // (robot), (parcel, path)
    active_paths: Vec<Option<(usize, Option<PathType>)>>,
    // parcel
    active_requests: FnvHashSet<usize>,
}

enum PathType {
    Pickup(Path),
    Delivery(Path),
}

impl<'p, 's, 'a> GreedyShortestPaths<'p, 's, 'a> {
    /// Requires up-to-date assignments
    fn update_paths(&mut self, requests: &FnvHashMap<usize, Request>, last_state: &State) {
        for (robot, task) in self.active_paths.iter_mut().enumerate() {
            if let (None, Some(parcel)) = (task, self.assignment[robot].pop()) {
                *task = Some((parcel, None));
            }

            if let Some((parcel, maybe_path)) = task {
                match maybe_path {
                    None => {
                        let current_vertex = last_state.robot_states[robot].vertex;
                        let Request { from, to, } = requests.get(&parcel).unwrap();
                        if current_vertex == *from {
                            if let Some(path) = self.time_graph.find_path(self.time + 1, *from, *to) {
                                self.time_graph.remove_path(&path);

                                *maybe_path = Some(PathType::Delivery(path));
                            }
                        } else {
                            if let Some(path) = self.time_graph.find_path(self.time, current_vertex, *from) {
                                *maybe_path = Some(PathType::Pickup(path));
                            }
                        }
                    },
                    Some(PathType::Pickup(path)) => if path.end_time() > self.time {
                        *maybe_path = None;
                    },
                    Some(PathType::Delivery(path)) => if path.end_time() > self.time {
                        *maybe_path = None;
                    },
                }
            }
        }
    }
    /// time >= 1
    fn get_robot_instruction(
        &self,
        robot_id: usize,
        previous_state: &State,
        instructions: &mut Instructions,
    ) {
        debug_assert!(self.active_paths.len() > 0);

        if let Some((parcel, maybe_path)) = self.active_paths[robot_id] {

        }

        let previous_location = previous_state.robot_states[robot_id].vertex;
        let current_state = self.active_paths[robot_id]
            .as_ref()
            .map(|(parcel, path)| (parcel, path.nodes[self.time - path.start_time]));
        match (previous_location, current_state) {
            (None, Some((&parcel, vertex))) => {
                instructions.placements.push(PlacementInstruction {
                    robot_id,
                    parcel,
                    vertex,
                });
            }
            (Some(previous_vertex), Some((_, vertex))) => {
                if previous_vertex != vertex {
                    debug_assert!(previous_vertex.distance(vertex) == 1);

                    instructions
                        .movements
                        .push(MoveInstruction { robot_id, vertex });
                }
            }
            (Some(vertex), None) => {
                instructions.removals.push(RemovalInstruction {
                    parcel: previous_state.robot_states[robot_id].parcel_id.unwrap(),
                    robot_id,
                    vertex,
                });
            }
            (None, None) => (),
        }
    }
    fn update_assignment(&mut self, requests: &FnvHashMap<usize, Request>) {
        let unassigned_requests = requests
            .iter()
            .filter(|&(id, _)| !self.active_requests.contains(id))
            .collect::<Vec<_>>();

        let availability = self.get_earliest_availability();
        self.assignment = self
            .assignment_algorithm
            .calculate_assignment(requests, availability);
    }
    fn get_earliest_availability(&self) -> Vec<usize> {
        self.active_paths
            .iter()
            .enumerate()
            .map(|(_, maybe_path)| match maybe_path {
                None => self.time,
                Some((request_id, path)) => path.start_time + path.nodes.len() - 1 + 2,
            })
            .collect::<Vec<_>>()
    }
}

impl<'p, 's, 'a> PathAlgorithm<'p, 's, 'a> for GreedyShortestPaths<'p, 's, 'a> {
    fn instantiate(
        plan: &'p impl Plan,
        settings: &'s Settings,
        assignment_algorithm: Box<impl AssignmentAlgorithm<'p, 's> + 'a>,
    ) -> GreedyShortestPaths<'p, 's, 'a> {
        GreedyShortestPaths {
            time_graph: TimeGraph::from_plan(plan, settings.total_time),
            settings,
            assignment_algorithm,

            time: 1,
            assignment: repeat(Vec::with_capacity(0))
                .take(settings.maximum_robots)
                .collect(),
            active_paths: repeat(None).take(settings.maximum_robots).collect(),
            active_requests: FnvHashSet::default(),
        }
    }
    fn initialize(&mut self) -> Result<(), NoSolutionError> {
        Ok(())
    }
    fn next_step(&mut self, history: &History) -> Instructions {
        self.time_graph.clean_front(self.time);

        if self.contains_new_requests(history) {
            self.update_assignment(&history.last_state().requests);
        }
        self.update_paths(&history.last_state().requests, history.last_state());

        let mut instructions = Instructions {
            movements: Vec::new(),
            placements: Vec::new(),
            removals: Vec::new(),
        };

        for robot in &history.last_state().robot_states {
            self.get_robot_instruction(robot.robot_id, history.last_state(), &mut instructions);
        }

        self.time += 1;
        instructions
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Path {
    pub start_time: usize,
    pub nodes: Vec<Vertex>,
}
impl Path {
    fn length(&self) -> usize {
        self.nodes.len()
    }
    fn end_time(&self) -> usize {
        self.start_time + self.nodes.len() - 1
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use algorithm::assignment::greedy_makespan::GreedyMakespan;
    use simulation::plan::one_three_rectangle::OneThreeRectangle;

    #[test]
    fn test_calculate_paths_single_3_3() {
        let plan = OneThreeRectangle::new(3, 3);
        let settings = Settings {
            total_time: 10,
            maximum_robots: 1,
            nr_requests: 1,
            real_time: false,
            output_file: None,
        };
        let mut requests = FnvHashMap::default();
        let source = Vertex { x: 0, y: 1 };
        let terminal = Vertex { x: 2, y: 1 };
        requests.insert(
            0,
            Request {
                from: source,
                to: terminal,
            },
        );
        let mut assignment_algorithm = Box::new(GreedyMakespan::instantiate(&plan, &settings));
        let mut algorithm =
            GreedyShortestPaths::instantiate(&plan, &settings, assignment_algorithm);
        algorithm.update_assignment(&requests);
        algorithm.update_paths(&requests);
        assert_eq!(
            algorithm.active_paths,
            vec![Some((
                0,
                Path {
                    start_time: 1,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                },
            ))]
        );
    }

    #[test]
    fn test_calculate_paths_single_5_5() {
        let plan = OneThreeRectangle::new(5, 5);
        let settings = Settings {
            total_time: 10,
            maximum_robots: 1,
            nr_requests: 1,
            output_file: None,
        };
        let mut assignment_algorithm = Box::new(GreedyMakespan::instantiate(&plan, &settings));
        let mut algorithm =
            GreedyShortestPaths::instantiate(&plan, &settings, assignment_algorithm);
        let mut requests = FnvHashMap::default();
        let parcel_id = 0;
        let from = Vertex { x: 0, y: 3 };
        let to = Vertex { x: 1, y: 0 };
        requests.insert(parcel_id, Request { from, to });

        algorithm.update_assignment(&requests);
        algorithm.update_paths(&requests);
        let paths = algorithm.active_paths;
        assert_eq!(paths.len(), 1);
        assert!(paths[0].is_some());
        let (parcel, path) = paths[0].as_ref().unwrap();
        assert_eq!(*parcel, parcel_id);
        assert_eq!(path.start_time, 1);
        assert_eq!(path.nodes.first(), Some(&from));
        assert_eq!(path.nodes.last(), Some(&to));
    }

    #[test]
    fn test_calculate_paths_two_same() {
        let plan = OneThreeRectangle::new(3, 3);
        let mut requests = FnvHashMap::default();
        let source = Vertex { x: 0, y: 1 };
        let terminal = Vertex { x: 2, y: 1 };
        requests.insert(
            0,
            Request {
                from: source,
                to: terminal,
            },
        );
        requests.insert(
            1,
            Request {
                from: source,
                to: terminal,
            },
        );
        let settings = Settings {
            total_time: 10,
            maximum_robots: 2,
            nr_requests: requests.len() as u64,
            output_file: None,
        };
        let mut assignment_algorithm = Box::new(GreedyMakespan::instantiate(&plan, &settings));
        let mut algorithm =
            GreedyShortestPaths::instantiate(&plan, &settings, assignment_algorithm);

        algorithm.update_assignment(&requests);
        algorithm.update_paths(&requests);

        let expected = vec![
            Some((
                1,
                Path {
                    start_time: 1,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                },
            )),
            None,
        ];
        assert_eq!(algorithm.active_paths, expected);
    }

    #[test]
    fn test_calculate_paths_two_cross() {
        let plan = OneThreeRectangle::new(3, 3);
        let mut requests = FnvHashMap::default();
        let source = Vertex { x: 0, y: 1 };
        let terminal = Vertex { x: 2, y: 1 };
        requests.insert(
            0,
            Request {
                from: source,
                to: terminal,
            },
        );
        requests.insert(
            1,
            Request {
                from: source,
                to: terminal,
            },
        );
        let settings = Settings {
            total_time: 10,
            maximum_robots: 2,
            nr_requests: requests.len() as u64,
            output_file: None,
        };
        let mut assignment_algorithm = Box::new(GreedyMakespan::instantiate(&plan, &settings));
        let mut algorithm =
            GreedyShortestPaths::instantiate(&plan, &settings, assignment_algorithm);

        algorithm.update_assignment(&requests);
        algorithm.update_paths(&requests);
        let expected = vec![
            Some((
                1,
                Path {
                    start_time: 1,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                },
            )),
            None,
        ];
        assert_eq!(algorithm.active_paths, expected);
    }
}
