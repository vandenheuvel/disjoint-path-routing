use simulation::demand::Request;
use simulation::plan::Vertex;
use simulation::statistics::Statistics;
use std::collections::HashMap;

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
    pub fn empty() -> History {
        History {
            states: Vec::new(),
            calculation_times: Vec::new(),
        }
    }
    pub fn calculate_statistics(&self) -> Statistics {
        Statistics {}
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
