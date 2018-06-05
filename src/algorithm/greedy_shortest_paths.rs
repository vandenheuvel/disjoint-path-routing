use algorithm::time_graph::TimeGraph;
use algorithm::Algorithm;
use algorithm::NoSolutionError;
use fnv::FnvHashMap;
use priority_queue::PriorityQueue;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::plan::Vertex;
use simulation::settings::Settings;
use simulation::state::History;
use simulation::Instructions;
use simulation::MoveInstruction;
use simulation::PlacementInstruction;
use simulation::RemovalInstruction;
use std::cmp::Reverse;

pub struct GreedyShortestPaths<'p, 's> {
    // Initialized at instantiation
    time_graph: TimeGraph<'p>,
    settings: &'s Settings,

    // (robot), (parcel, path)
    paths: Vec<Vec<(usize, Path)>>,
}

impl<'p, 's> GreedyShortestPaths<'p, 's> {
    fn calculate_paths(
        &mut self,
        requests: &FnvHashMap<usize, Request>,
    ) -> Result<FnvHashMap<usize, Vec<(usize, Path)>>, NoSolutionError> {
        let mut paths =
            FnvHashMap::with_capacity_and_hasher(self.settings.maximum_robots, Default::default());
        let mut robot_availability = (0..self.settings.maximum_robots)
            .map(|robot| (robot, Reverse(1 /* We start at time 1 */ as usize)))
            .collect::<PriorityQueue<_, _>>();

        for (&id, &Request { from, to }) in requests.into_iter() {
            let (&robot, &Reverse(earliest_available)) = robot_availability.peek().unwrap();
            let possible_path =
                self.time_graph
                    .find_earliest_path_after(earliest_available, from, to);
            if let Some(path) = possible_path {
                self.time_graph.remove_path(&path);
                robot_availability.push(robot, Reverse(path.start_time + (path.length() - 1) + 2));
                if !paths.contains_key(&robot) {
                    paths.insert(robot, Vec::new());
                }
                paths.get_mut(&robot).unwrap().push((id, path));
            } else {
                return Err(NoSolutionError::new(
                    format!("No path found for request {}", id).to_string(),
                ));
            }
        }

        Ok(paths)
    }
    fn sort_paths_by_robot(
        &mut self,
        mut paths: FnvHashMap<usize, Vec<(usize, Path)>>,
    ) -> Vec<Vec<(usize, Path)>> {
        for robot in 0..self.settings.maximum_robots {
            if !paths.contains_key(&robot) {
                paths.insert(robot, Vec::with_capacity(0));
            }
        }
        let mut lookup = paths.into_iter().collect::<Vec<_>>();
        lookup.sort_by_key(|&(robot, _)| robot);
        let mut lookup = lookup
            .into_iter()
            .map(|(_, paths)| paths)
            .collect::<Vec<_>>();
        for paths in lookup.iter_mut() {
            paths.sort_by_key(|&(_, ref path)| path.start_time);
        }

        lookup
    }
    /// time >= 1
    fn get_robot_instruction(&self, robot_id: usize, time: usize, instructions: &mut Instructions) {
        debug_assert!(self.paths.len() > 0);

        let previous_state = self.paths[robot_id].iter().find(|&(_, path)| {
            path.start_time <= time - 1 && time - 1 <= path.start_time + path.length() - 1
        });
        let current_state = self.paths[robot_id].iter().find(|&(_, path)| {
            path.start_time <= time && time <= path.start_time + path.length() - 1
        });
        match (previous_state, current_state) {
            (None, Some(&(parcel, Path { start_time, ref nodes, }, )), ) => {
                instructions.placements.push(PlacementInstruction {
                    robot_id,
                    parcel,
                    vertex: nodes[time - start_time],
                });
            }
            (Some(then), Some(now)) => {
                debug_assert!(then == now);

                let &(
                    _,
                    Path {
                        start_time,
                        ref nodes,
                    },
                ) = now;

                let new_node = nodes[time - start_time];
                let previous_node = nodes[time - start_time - 1];
                if previous_node != new_node {
                    debug_assert!(previous_node.distance(new_node) == 1);

                    instructions.movements.push(MoveInstruction {
                        robot_id,
                        vertex: new_node,
                    });
                }
            }
            (Some(&(parcel, Path { start_time, ref nodes, }, )), None, ) => {
                instructions.removals.push(RemovalInstruction {
                    parcel,
                    robot_id,
                    vertex: nodes[time - start_time - 1],
                });
            }
            (None, None) => (),
        }
    }
}

impl<'p, 's> Algorithm<'p, 's> for GreedyShortestPaths<'p, 's> {
    fn instantiate(plan: &'p impl Plan, settings: &'s Settings) -> GreedyShortestPaths<'p, 's> {
        let time_graph = TimeGraph::from_plan(plan, settings.total_time);

        GreedyShortestPaths {
            time_graph,
            settings,

            paths: Vec::with_capacity(0),
        }
    }
    fn initialize(&mut self, requests: &FnvHashMap<usize, Request>) -> Result<(), NoSolutionError> {
        debug_assert!(requests.len() > 0);

        let paths = self.calculate_paths(requests)?;
        self.paths = self.sort_paths_by_robot(paths);
        Ok(())
    }
    fn next_step(&mut self, history: &History) -> Instructions {
        debug_assert!(self.paths.len() > 0);

        let mut instructions = Instructions {
            movements: Vec::new(),
            placements: Vec::new(),
            removals: Vec::new(),
        };

        for robot in &history.last_state().robot_states {
            self.get_robot_instruction(robot.robot_id, history.time(), &mut instructions);
        }

        instructions
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct Path {
    pub start_time: usize,
    pub nodes: Vec<Vertex>,
}
impl Path {
    fn length(&self) -> usize {
        self.nodes.len()
    }
}

#[cfg(test)]
mod test {
    use super::*;
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
        let mut algorithm = GreedyShortestPaths::instantiate(&plan, &settings);
        let paths = algorithm.calculate_paths(&requests).ok().unwrap();
        let paths = algorithm.sort_paths_by_robot(paths);
        assert_eq!(paths.len(), 1);
        assert_eq!(
            paths[0],
            vec![(
                0,
                Path {
                    start_time: 1,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                },
            )]
        );
    }

    #[test]
    fn test_calculate_paths_single_5_5() {
        let plan = OneThreeRectangle::new(5, 5);
        let settings = Settings {
            total_time: 10,
            maximum_robots: 1,
            nr_requests: 1,
            real_time: false,
            output_file: None,
        };
        let mut algorithm = Box::new(<GreedyShortestPaths as Algorithm>::instantiate(
            &plan, &settings,
        ));
        let mut requests = FnvHashMap::default();
        let parcel_id = 0;
        let from = Vertex { x: 0, y: 3 };
        let to = Vertex { x: 1, y: 0 };
        requests.insert(parcel_id, Request { from, to });

        let paths = algorithm.calculate_paths(&requests).ok().unwrap();
        let paths = algorithm.sort_paths_by_robot(paths);
        assert_eq!(paths.len(), 1);
        assert_eq!(paths[0][0].0, parcel_id);
        let path = &paths[0][0].1;
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
            real_time: false,
            output_file: None,
        };
        let mut algorithm = GreedyShortestPaths::instantiate(&plan, &settings);
        let paths = algorithm.calculate_paths(&requests).ok().unwrap();
        let paths = algorithm.sort_paths_by_robot(paths);
        let expected = vec![
            vec![(
                1,
                Path {
                    start_time: 1,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                },
            )],
            vec![(
                0,
                Path {
                    start_time: 3,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                },
            )],
        ];
        assert_eq!(paths, expected);
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
            real_time: false,
            output_file: None,
        };
        let mut algorithm = GreedyShortestPaths::instantiate(&plan, &settings);
        let paths = algorithm.calculate_paths(&requests).ok().unwrap();
        let paths = algorithm.sort_paths_by_robot(paths);
        let expected = vec![
            vec![(
                1,
                Path {
                    start_time: 1,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                },
            )],
            vec![(
                0,
                Path {
                    start_time: 3,
                    nodes: vec![source, Vertex { x: 1, y: 1 }, terminal],
                },
            )],
        ];
        assert_eq!(paths, expected);
    }
}
