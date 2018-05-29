use simulation::plan::Plan;
use simulation::settings::Settings;
use algorithm::Algorithm;
use simulation::demand::Demand;
use simulation::plan::Vertex;
use simulation::demand::Request;
use std::collections::HashSet;

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
    }
    fn set_initial_state(&mut self) {
        let mut robot_states = Vec::with_capacity(self.settings.maximum_robots);

        for robot in 0..self.settings.maximum_robots {
            robot_states.push(RobotState {
                robot,
                vertex: None,
                parcel: None,
            });
        }

        self.history.states.push(State {
            robot_states,
            requests: self.demand.generate(self.plan, self.settings.nr_requests),
        });
    }
    pub fn run(&mut self) {
        while self.history.time() < self.settings.total_time {
            let instructions = self.algorithm.next_step(&self.history.states);
            self.new_state(instructions);
            break;
        }
    }
    fn new_state(&self, instructions: Instructions) -> Result<(), String> {
        let mut used_states = HashSet::new();
        let mut new_states = self.history.last_state().robot_states.clone();

        for MoveInstruction { robot, parcel, vertex, } in instructions.movements {
            if !used_states.contains(&vertex) {
                new_states[robot] = RobotState { robot, parcel, vertex: Some(vertex), };
            }

            used_states.insert(vertex);
            if let Some(previous_vertex) = self.history.last_robot_state(robot).vertex {
                used_states.insert(previous_vertex);
            }
        }

        for PlacementInstruction { robot, parcel, vertex, } in instructions.placements {
            if !used_states.contains(&vertex) {
                new_states[robot] = RobotState { robot, parcel: Some(parcel), vertex: Some(vertex), };
            }

            used_states.insert(vertex);
            if let Some(previous_vertex) = self.history.last_robot_state(robot).vertex {
                used_states.insert(previous_vertex);
            }
        }

        for RemovalInstruction { robot, parcel, vertex, } in instructions.removals {
            if !used_states.contains(&vertex) {
                new_states[robot] = RobotState { robot, parcel: None, vertex: None, };
            }

            if let Some(previous_vertex) = self.history.last_robot_state(robot).vertex {
                used_states.insert(previous_vertex);
            }
        }

        Ok(())
    }
}

pub struct State {
    pub robot_states: Vec<RobotState>,
    pub requests: Vec<Request>,
}

#[derive(Copy, Clone)]
pub struct RobotState {
    pub robot: usize,
    pub parcel: Option<Request>,
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
    fn calculate_statistics(&self) -> Statistics {
        Statistics {
        }
    }
    fn last_state(&self) -> &State {
        self.states.last().unwrap()
    }
    fn last_robot_state(&self, robot: usize) -> RobotState {
        self.last_state().robot_states[robot]
    }
    fn time(&self) -> usize {
        self.states.len()
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
    pub robot: usize,
    pub parcel: Option<Request>,
    pub vertex: Vertex,
}
pub struct ParcelInstruction {
    pub robot: usize,
    pub parcel: Request,
    pub vertex: Vertex,
}
pub type PlacementInstruction = ParcelInstruction;
pub type RemovalInstruction = ParcelInstruction;
