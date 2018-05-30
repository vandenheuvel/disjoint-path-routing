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
use std::collections::HashMap;

pub struct GreedyShortestPaths<'p, 's> {
    // Initialized at instantiation
    time_graph: TimeGraph<'p>,
    settings: &'s Settings,

    // After
    /// One for each request => Path
    paths: Option<HashMap<usize, Path>>,
}

impl<'p, 's> GreedyShortestPaths<'p, 's> {
    fn calculate_paths(&mut self, requests: &HashMap<usize, Request>) {
        let mut paths = HashMap::with_capacity(requests.len());

        for (&id, &Request { source, terminal, }) in requests.into_iter() {
            let path = self.time_graph.find_earliest_path(source, terminal);
            self.time_graph.remove_path(&path);
            paths.insert(id, path);
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

            for (robot_id, robot_state) in state.robot_states.iter().enumerate() {
                match robot_state.parcel_id {
                    Some(parcel) => {
                        match robot_state.vertex {
                            Some(vertex) if vertex == *paths.get(&parcel).unwrap().nodes.last().unwrap() =>
                                removals.push(RemovalInstruction { robot_id, vertex, parcel, }),
                            Some(vertex) => {
                                let &Path { started, start_time, nodes, } = &paths.get(&parcel).unwrap();
                                let next = nodes[current_time - start_time];
                                movements.push(MoveInstruction { robot_id, vertex, parcel: Some(parcel), });
                            }
                            None => panic!("Should be placed if it has a parcel"),
                        }
                    },
                    None => {
                        let mut mark_started = None;
                        for (&parcel, Path { started, start_time, nodes, }) in paths.iter() {
                            if *start_time == current_time && !started {
                                placements.push(PlacementInstruction {
                                    robot_id: robot_state.robot_id,
                                    parcel,
                                    vertex: *nodes.first().unwrap(),
                                });
                            }

                            mark_started = Some(parcel);
                            break;
                        }
                        if let Some(parcel) = mark_started {
                            paths.get_mut(&parcel).unwrap().started = true;
                        }
                    },
                }
            }

            Instructions { movements, placements, removals, }
        } else { panic!("We should have just filled these!?"); }
    }
}

pub struct Path {
    pub started: bool,
    pub start_time: usize,
    pub nodes: Vec<Vertex>
}
