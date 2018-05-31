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
use simulation::simulation::RobotState;
use std::collections::HashSet;
use simulation::simulation::History;

pub struct GreedyShortestPaths<'p, 's> {
    // Initialized at instantiation
    time_graph: TimeGraph<'p>,
    settings: &'s Settings,

    // After
    /// One for each request => Path
    paths: HashMap<usize, Path>,
}

impl<'p, 's> GreedyShortestPaths<'p, 's> {
    fn calculate_paths(&mut self, requests: &HashMap<usize, Request>) -> HashMap<usize, Path> {
        let mut paths = HashMap::with_capacity(requests.len());

        for (&id, &Request { source, terminal, }) in requests.into_iter() {
            let path = self.time_graph.find_earliest_path(source, terminal);
            self.time_graph.remove_path(&path);
            paths.insert(id, path);
        }

        paths
    }
    fn move_or_remove_robot(&self,
                            robot_id: usize,
                            vertex: Vertex,
                            parcel: usize,
                            instructions: &mut Instructions,
                            current_time: usize) {
        if vertex == *self.paths.get(&parcel).unwrap().nodes.last().unwrap() {
            instructions.removals.push(RemovalInstruction { robot_id, vertex, parcel, });
        } else {
            let &Path { start_time, ref nodes, } = self.paths.get(&parcel).unwrap();
            let next = nodes[current_time - start_time];
            instructions.movements.push(MoveInstruction {
                robot_id,
                vertex: next,
                parcel: Some(parcel),
            });
        }
    }
    fn place_new_parcel(&self,
                        robot_id: usize,
                        paths_started: &mut HashSet<usize>,
                        instructions: &mut Instructions,
                        current_time: usize) {
        let new_path = self.paths.iter()
            .find(|&(parcel, Path { start_time, nodes, })| {
                *start_time == current_time && !paths_started.contains(parcel)
            });
        if let Some((parcel, Path { start_time, nodes, })) = new_path {
            instructions.placements.push(PlacementInstruction {
                robot_id,
                parcel: *parcel,
                vertex: *nodes.first().unwrap(),
            });
            paths_started.insert(*parcel);
        }
    }
    fn update_robot_state(&self,
                          robot_state: &RobotState,
                          instructions: &mut Instructions,
                          paths_started: &mut HashSet<usize>,
                          current_time: usize) {
        match robot_state.parcel_id {
            Some(parcel) => match robot_state.vertex {
                Some(vertex) => self.move_or_remove_robot(robot_state.robot_id, vertex, parcel, instructions, current_time),
                None => panic!("Robots with a parcel must be placed on a vertex"),
            },
            None => self.place_new_parcel(robot_state.robot_id, paths_started, instructions, current_time),
        }
    }
}

impl<'p, 's> Algorithm<'p, 's> for GreedyShortestPaths<'p, 's> {
    fn instantiate(plan: &'p impl Plan, settings: &'s Settings) -> GreedyShortestPaths<'p, 's> {
        let time_graph = TimeGraph::from_plan(plan, settings.total_time);

        GreedyShortestPaths {
            time_graph,
            settings,

            paths: HashMap::new(),
        }
    }
    fn initialize(&mut self, requests: &HashMap<usize, Request>) {
        debug_assert!(requests.len() > 0);

        self.paths = self.calculate_paths(requests);
    }
    fn next_step(&mut self, history: &History) -> Instructions {
        debug_assert!(self.paths.len() > 0);

        let mut paths_started = HashSet::new();
        let mut instructions = Instructions {
            movements: Vec::new(),
            placements: Vec::new(),
            removals: Vec::new(),
        };

        for robot_state in &history.last_state().robot_states {
            self.update_robot_state(robot_state, &mut instructions, &mut paths_started, history.time());
        }

        instructions
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct Path {
    pub start_time: usize,
    pub nodes: Vec<Vertex>
}

#[cfg(test)]
mod test {

    use super::*;
    use simulation::plan::OneThreeRectangle;

    #[test]
    fn test_calculate_paths() {
        let plan = OneThreeRectangle::new(3, 3);
        let settings = Settings {
            total_time: 10,
            maximum_robots: 1,
            nr_requests: 1,
        };
        let mut requests = HashMap::new();
        let source = Vertex { x: 0, y: 1, };
        let terminal = Vertex { x: 2, y: 1, };
        requests.insert(0, Request { source, terminal, });
        let mut algorithm = GreedyShortestPaths::instantiate(&plan, &settings);
        algorithm.calculate_paths(&requests);
        assert_eq!(algorithm.paths.len(), 1);
        assert_eq!(algorithm.paths.get(&0), Some(&Path {
            start_time: 0,
            nodes: vec![source, Vertex { x: 1, y: 1, }, terminal],
        }));
    }
}
