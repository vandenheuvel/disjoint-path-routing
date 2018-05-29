use algorithm::Algorithm;
use simulation::plan::Plan;
use simulation::settings::Settings;
use simulation::simulation::Instructions;
use simulation::simulation::State;
use algorithm::time_graph::TimeGraph;
use simulation::plan::Vertex;
use simulation::demand::Request;
use simulation::simulation::{MoveInstruction, PlacementInstruction};
use simulation::simulation::RemovalInstruction;

pub struct GreedyShortestPaths<'p, 's> {
    // Initialized at instantiation
    time_graph: TimeGraph<'p>,
    settings: &'s Settings,

    // After
    /// One for each request
    paths: Option<Vec<(bool, Path)>>,
}

impl<'p, 's> GreedyShortestPaths<'p, 's> {
    fn calculate_paths(&mut self, requests: &Vec<Request>) {
        let mut paths = Vec::with_capacity(requests.len());

        for request in requests {
            let path = self.time_graph.find_earliest_path(request.source, request.terminal);
            self.time_graph.remove_path(&path);
            paths.push((false, path));
        }

        self.paths = Some(paths);
    }
}

impl<'p, 's> Algorithm<'p, 's> for GreedyShortestPaths<'p, 's> {
    fn instantiate(plan: &'p Plan, settings: &'s Settings) -> GreedyShortestPaths<'p, 's> {
        let time_graph = TimeGraph::from_plan(plan, settings.total_time);

        GreedyShortestPaths {
            time_graph,
            settings,

            paths: None,
        }
    }
    fn next_step(&mut self, history: &Vec<State>) -> Instructions {
        let current_time = history.len();
        let state = history.last().unwrap();
        let (requests, states) = (&state.requests, &state.robot_states);

        if let None = self.paths {
            self.calculate_paths(requests);
        }
        if let Some(ref mut paths) = self.paths {
            let mut movements = Vec::new();
            let mut placements = Vec::new();
            let mut removals = Vec::new();

            for robot_state in &state.robot_states {
                match robot_state.parcel {
                    Some(parcel) => {
                        match robot_state.vertex {
                            Some(vertex) if vertex == parcel.terminal => removals.push(RemovalInstruction {
                                robot: robot_state.robot,
                                vertex,
                                parcel,
                            }),
                            Some(vertex) => {
                                let (_, (start_time, nodes)) = &paths[parcel.id as usize];
                                let next = nodes[current_time - start_time];
                                movements.push(MoveInstruction {
                                    robot: robot_state.robot,
                                    vertex,
                                    parcel: Some(parcel),
                                });
                            }
                            None => panic!("Should be placed if it has a parcel"),
                        }
                    },
                    None => {
                        let mut mark_started = None;
                        for (parcel, &(started, (start_time, ref nodes))) in paths.iter().enumerate() {
                            if start_time == current_time && !started {
                                placements.push(PlacementInstruction {
                                    robot: robot_state.robot,
                                    vertex: *nodes.first().unwrap(),
                                    parcel: requests[parcel],
                                });
                            }

                            mark_started = Some(parcel);
                            break;
                        }
                        if let Some(parcel) = mark_started {
                            paths[parcel].0 = true;
                        }
                    },
                }
            }

            Instructions { movements, placements, removals, }
        } else { panic!("We should have just filled these!?"); }
    }
}

pub type Path = (usize, Vec<Vertex>);
