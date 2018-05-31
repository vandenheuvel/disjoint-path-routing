use simulation::plan::Plan;
use simulation::settings::Settings;
use algorithm::Algorithm;
use simulation::demand::Demand;
use simulation::plan::Vertex;
use simulation::demand::Request;
use std::collections::HashSet;
use std::collections::HashMap;

pub struct Simulation<'a, 'p, 's> {
    algorithm: Box<Algorithm<'p, 's> + 'a>,
    plan: &'p Plan,
    demand: Box<Demand + 'a>,
    settings: &'s Settings,

    history: History,
}

impl<'a, 'p, 's> Simulation<'a, 'p, 's> {
    pub fn new(algorithm: Box<Algorithm<'p, 's> + 'a>,
           plan: &'p Plan,
           demand: Box<Demand + 'a>,
           settings: &'s Settings) -> Simulation<'a, 'p, 's> {

        Simulation {
            algorithm,
            plan,
            demand,
            settings,

            history: History::empty(),
        }
    }
    /// Gets the initial state of the system set up, creates history of time 0.
    pub fn initialize(&mut self) {
        self.set_initial_state();
        self.algorithm.initialize(&self.history.last_state().requests);
    }
    fn set_initial_state(&mut self) {
        let mut robot_states = Vec::with_capacity(self.settings.maximum_robots);
        for robot in 0..self.settings.maximum_robots {
            robot_states.push(RobotState {
                robot_id: robot,
                vertex: None,
                parcel_id: None,
            });
        }
        let requests = self.demand
            .generate(self.plan, self.settings.nr_requests)
            .into_iter().enumerate().collect();

        self.history.states.push(State {
            robot_states,
            requests,
        });
    }
    pub fn run(&mut self) {
        while self.history.last_state().requests.len() > 0 &&
            self.history.time() < self.settings.total_time {
            let instructions = self.algorithm.next_step(&self.history);
            self.new_state(instructions);
        }

        println!("{:?}", self.history.time());
    }
    fn new_state(&mut self, instructions: Instructions) -> Result<(), String> {
        let mut used_states = HashSet::new();
        let mut new_states = self.history.last_state().robot_states.clone();
        let mut new_requests = self.history.last_state().requests.clone();

        for MoveInstruction { robot_id, parcel, vertex, } in instructions.movements {
            if !used_states.contains(&vertex) {
                new_states[robot_id] = RobotState { robot_id, parcel_id: parcel, vertex: Some(vertex), };
            }

            used_states.insert(vertex);
            if let Some(previous_vertex) = self.history.last_robot_state(robot_id).vertex {
                used_states.insert(previous_vertex);
            }
        }

        for PlacementInstruction { robot_id, parcel, vertex, } in instructions.placements {
            if !used_states.contains(&vertex) {
                new_states[robot_id] = RobotState { robot_id, parcel_id: Some(parcel), vertex: Some(vertex), };
            }

            used_states.insert(vertex);
            if let Some(previous_vertex) = self.history.last_robot_state(robot_id).vertex {
                used_states.insert(previous_vertex);
            }
        }

        for RemovalInstruction { robot_id, parcel, vertex, } in instructions.removals {
            if !used_states.contains(&vertex) {
                new_states[robot_id] = RobotState { robot_id, parcel_id: None, vertex: None, };
            }
            new_requests.remove(&parcel);

            if let Some(previous_vertex) = self.history.last_robot_state(robot_id).vertex {
                used_states.insert(previous_vertex);
            }
        }

        Ok(self.history.states.push(State { robot_states: new_states, requests: new_requests, }))
    }
}

pub struct State {
    pub robot_states: Vec<RobotState>,
    pub requests: HashMap<usize, Request>,
}

#[derive(Copy, Clone)]
pub struct RobotState {
    pub robot_id: usize,
    pub parcel_id: Option<usize>,
    pub vertex: Option<Vertex>,
}

pub struct History {
    pub states: Vec<State>,
    pub calculation_times: Vec<f64>,
}

impl History {
    fn empty() -> History {
        History {
            states: Vec::new(),
            calculation_times: Vec::new(),
        }
    }
    pub fn calculate_statistics(&self) -> Statistics {
        Statistics {
        }
    }
    pub fn last_state(&self) -> &State {
        self.states.last().unwrap()
    }
    pub fn last_robot_state(&self, robot: usize) -> RobotState {
        self.last_state().robot_states[robot]
    }
    pub fn time(&self) -> usize {
        self.states.len() - 1
    }
}

pub struct Statistics {
}

pub struct Instructions {
    pub movements: Vec<MoveInstruction>,
    pub placements: Vec<PlacementInstruction>,
    pub removals: Vec<RemovalInstruction>,
}
pub struct MoveInstruction {
    pub robot_id: usize,
    pub parcel: Option<usize>,
    pub vertex: Vertex,
}
pub struct ParcelInstruction {
    pub robot_id: usize,
    pub parcel: usize,
    pub vertex: Vertex,
}
pub type PlacementInstruction = ParcelInstruction;
pub type RemovalInstruction = ParcelInstruction;
