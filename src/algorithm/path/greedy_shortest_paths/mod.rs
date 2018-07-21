use algorithm::assignment::AssignmentAlgorithm;
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
use algorithm::path::greedy_shortest_paths::time_graph::TimeGraph;
use simulation::RobotRemovalInstruction;

pub mod time_graph;

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

#[derive(Clone, Debug, PartialEq, Eq)]
enum PathType {
    Pickup(Path),
    Delivery(Path),
}

impl<'p, 's, 'a> GreedyShortestPaths<'p, 's, 'a> {
    pub fn new(
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
                .take(settings.nr_robots)
                .collect(),
            active_paths: repeat(None).take(settings.nr_robots).collect(),
            active_requests: FnvHashSet::default(),
        }
    }
    /// Requires up-to-date assignments
    fn update_paths(&mut self, last_state: &State) {
        for (robot, mut task) in self.active_paths.iter_mut().enumerate() {
            if let (None, Some(parcel)) = (task.clone(), self.assignment[robot].pop()) {
                *task = Some((parcel, None));
            }

            let placed = last_state.robot_states[robot].vertex;
            if let (Some(current_vertex), Some((parcel, maybe_path))) = (placed, task) {
                match maybe_path {
                    None => {
                        let Request { from, to, } = last_state.requests.get(&parcel).unwrap();
                        if current_vertex == *from {
                            if let Some(path) = self.time_graph.find_path(self.time + 1, *from, *to) {
                                self.time_graph.remove_path(&path);
                                *maybe_path = Some(PathType::Delivery(path));
                            }
                        } else {
                            if let Some(path) = self.time_graph.find_path(self.time, current_vertex, *from) {
                                self.time_graph.remove_path(&path);
                                *maybe_path = Some(PathType::Pickup(path));
                            }
                        }
                    },
                    Some(PathType::Pickup(path)) => if self.time > path.end_time() {
                        println!("{:?}, {:?}", path, (path.end_time(), self.time));
                        *maybe_path = None;
                    },
                    Some(PathType::Delivery(path)) => if self.time > path.end_time() {
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

        let previous_vertex = previous_state.robot_states[robot_id].vertex.unwrap();
        if let &Some((parcel, ref maybe_path)) = &self.active_paths[robot_id] {
            match maybe_path {
                Some(PathType::Pickup(path)) => {
                    debug_assert!(self.time >= path.start_time);

                    if self.time == path.end_time() {
                        instructions.placements.push(PlacementInstruction {
                            robot_id,
                            parcel,
                            vertex: previous_vertex,
                        });
                    } else {
                        debug_assert!(self.time < path.end_time());

                        let next_state_location = path.nodes[1 + self.time - path.start_time];
                        if  previous_vertex != next_state_location {
                            debug_assert!(previous_vertex.distance(next_state_location) == 1);
                            instructions.movements.push(MoveInstruction {
                                robot_id,
                                vertex: next_state_location,
                            });
                        }
                    }
                },
                Some(PathType::Delivery(path)) => {
                    if self.time == path.end_time() {
                        instructions.removals.push(RemovalInstruction {
                            robot_id,
                            parcel,
                            vertex: previous_vertex,
                        });
                    } else {
                        debug_assert!(self.time < path.end_time());

                        let next_state_location = path.nodes[1 + self.time - path.start_time];
                        if  previous_vertex != next_state_location {
                            debug_assert!(previous_vertex.distance(next_state_location) == 1);
                            instructions.movements.push(MoveInstruction {
                                robot_id,
                                vertex: next_state_location,
                            });
                        }
                    }
                },
                None => instructions.robot_removeals.push(RobotRemovalInstruction {
                    robot_id,
                    vertex: previous_vertex,
                }),
            }
        }
    }
    fn update_assignment(&mut self, state: &State) {
        let unassigned_requests = state.requests
            .iter()
            .filter(|&(id, _)| !self.active_requests.contains(id))
            .collect::<Vec<_>>();

        let availability = self.get_earliest_availability(state);
        self.assignment = self
            .assignment_algorithm
            .calculate_assignment(&state.requests, &availability);
    }
    fn get_earliest_availability(&self, state: &State) -> Vec<(usize, Vertex)> {
        self.active_paths
            .iter()
            .enumerate()
            .map(|(robot, maybe_path)| match maybe_path {
                None => (self.time, state.robot_states[robot].vertex.unw),
                Some((request_id, maybe_path)) => match maybe_path {
                    Some(path) => {
                        let path = match path {
                            PathType::Pickup(path) => path,
                            PathType::Delivery(path) => path,
                        };
                        (path.start_time + path.nodes.len() - 1 + 2, *path.nodes.last().unwrap())
                    },
                    None => (self.time, state.robot_states[robot].vertex),
                }
            })
            .collect::<Vec<_>>()
    }
}

impl<'p, 's, 'a> PathAlgorithm<'p, 's, 'a> for GreedyShortestPaths<'p, 's, 'a> {
    fn initialize(&mut self) -> Result<(), NoSolutionError> {
        Ok(())
    }
    fn next_step(&mut self, history: &History) -> Instructions {
        self.time_graph.clean_front(self.time);

        if self.contains_new_requests(history) {
            self.update_assignment(history.last_state());
        }
        self.update_paths(history.last_state());

        let mut instructions = Instructions {
            movements: Vec::new(),
            placements: Vec::new(),
            removals: Vec::new(),
        };

        for robot in &history.last_state().robot_states {
            if history.last_robot_state(robot).vertex.is_some() {
                self.get_robot_instruction(robot.robot_id, history.last_state(), &mut instructions);
            }
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

    use algorithm::assignment::greedy_makespan::GreedyMakespan;
    use simulation::plan::one_three_rectangle::OneThreeRectangle;
    use simulation::settings::Settings;
    use simulation::demand::Request;
    use simulation::plan::Vertex;
    use fnv::FnvHashMap;
    use algorithm::path::greedy_shortest_paths::Path;
    use algorithm::path::greedy_shortest_paths::GreedyShortestPaths;
    use simulation::state::State;
    use simulation::state::RobotState;
    use algorithm::path::greedy_shortest_paths::PathType;

    #[test]
    fn test_calculate_paths_single_3_3() {
        let plan = OneThreeRectangle::new(3, 3);
        let settings = Settings {
            total_time: 10,
            nr_robots: 1,
            nr_requests: 1,
            output_file: None,
        };
        let source = Vertex { x: 0, y: 1 };
        let terminal = Vertex { x: 2, y: 1 };
        let requests = map![
            0 => Request {
                from: source,
                to: terminal,
            },
        ];
        let state = State {
            robot_states: vec![RobotState {
                robot_id: 0,
                parcel_id: None,
                vertex: Vertex { x: 0, y: 1, },
            }],
            requests,
        };
        let mut assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
        let mut algorithm =
            GreedyShortestPaths::new(&plan, &settings, assignment_algorithm);
        algorithm.update_assignment(&state);
        algorithm.update_paths(&state);
        assert_eq!(
            algorithm.active_paths,
            vec![Some((
                0 as usize,
                Some(PathType::Delivery(Path {
                    start_time: 2,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                })),
            ))]
        );
    }

    #[test]
    fn test_calculate_paths_single_5_5() {
        let plan = OneThreeRectangle::new(5, 5);
        let settings = Settings {
            total_time: 10,
            nr_robots: 1,
            nr_requests: 1,
            output_file: None,
        };
        let mut assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
        let mut algorithm = GreedyShortestPaths::new(&plan, &settings, assignment_algorithm);

        let from = Vertex { x: 0, y: 3, };
        let to = Vertex { x: 1, y: 0, };
        let requests = map![
            0 => Request { from, to, },
        ];
        let state = State {
            robot_states: vec![RobotState {
                robot_id: 0,
                parcel_id: None,
                vertex: Vertex { x: 0, y: 1, },
            }],
            requests,
        };

        algorithm.update_assignment(&state);
        algorithm.update_paths(&state);
        assert_eq!(
            algorithm.active_paths,
            vec![Some((
                0 as usize,
                Some(PathType::Pickup(Path {
                    start_time: 1,
                    nodes: vec![
                        Vertex { x: 0, y: 1, },
                        Vertex { x: 0, y: 2 },
                        from
                    ],
                })),
            ))]
        );
    }

    #[test]
    fn test_calculate_paths_two_same() {
        let plan = OneThreeRectangle::new(3, 3);
        let source = Vertex { x: 0, y: 1 };
        let terminal = Vertex { x: 2, y: 1 };
        let requests = map![
            0 => Request {
                from: source,
                to: terminal,
            },
            1 => Request {
                from: source,
                to: terminal,
            },
        ];
        let settings = Settings {
            total_time: 10,
            nr_robots: 2,
            nr_requests: requests.len() as u64,
            output_file: None,
        };
        let state = State {
            robot_states: vec![
                RobotState {
                    robot_id: 0,
                    parcel_id: None,
                    vertex: Vertex { x: 0, y: 1, },
                },
                RobotState {
                    robot_id: 1,
                    parcel_id: None,
                    vertex: Vertex { x: 0, y: 0, },
                }],
            requests,
        };
        let mut assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
        let mut algorithm =
            GreedyShortestPaths::new(&plan, &settings, assignment_algorithm);

        algorithm.update_assignment(&state);
        algorithm.update_paths(&state);

        let expected = vec![
            Some((
                1,
                Some(PathType::Delivery(Path {
                    start_time: 2,
                    nodes: vec![Vertex { x: 0, y: 1, }, Vertex { x: 1, y: 1 }, terminal],
                })),
            )),
            Some((
                0,
                Some(PathType::Pickup(Path {
                    start_time: 1,
                    nodes: vec![
                        Vertex { x: 0, y: 0, },
                        Vertex { x: 0, y: 0, },
                        Vertex { x: 0, y: 0, },
                        source,
                    ],
                })),
            )),
        ];
        assert_eq!(algorithm.active_paths, expected);
    }

    #[test]
    fn test_calculate_paths_two_cross() {
        let plan = OneThreeRectangle::new(3, 3);
        let source = Vertex { x: 0, y: 1 };
        let terminal = Vertex { x: 2, y: 1 };
        let requests = map![
            0 => Request {
                from: source,
                to: terminal,
            },
            1 => Request {
                from: source,
                to: terminal,
            },
        ];
        let settings = Settings {
            total_time: 10,
            nr_robots: 2,
            nr_requests: requests.len() as u64,
            output_file: None,
        };
        let state = State {
            robot_states: vec![
                RobotState {
                    robot_id: 0,
                    parcel_id: None,
                    vertex: Vertex { x: 0, y: 1, },
                },
               RobotState {
                   robot_id: 1,
                   parcel_id: None,
                   vertex: Vertex { x: 0, y: 1, },
               }
            ],
            requests,
        };
        let mut assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
        let mut algorithm =
            GreedyShortestPaths::new(&plan, &settings, assignment_algorithm);

        algorithm.update_assignment(&state);
        algorithm.update_paths(&state);
        let expected = vec![
            Some((
                1,
                Some(PathType::Delivery(Path {
                    start_time: 2,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                })),
            )),
            Some((0, None)),
        ];
        assert_eq!(algorithm.active_paths, expected);
    }
}
