use fnv::FnvHashMap;
use simulation::demand::Request;
use simulation::plan::Vertex;
use simulation::statistics::Statistics;
use std::fs::File;
use std::io;
use std::io::BufWriter;
use std::io::Write;

#[derive(Debug, Eq, PartialEq)]
pub struct State {
    pub robot_states: Vec<RobotState>,
    pub requests: FnvHashMap<usize, Request>,
}

impl State {
    pub fn write(&self, writer: &mut BufWriter<File>) -> io::Result<()> {
        writer.write(format!("# Robot positions\n").as_bytes())?;
        for RobotState {
            robot_id,
            parcel_id,
            vertex,
        } in &self.robot_states
        {
            match (parcel_id, vertex) {
                (Some(parcel), Vertex { x, y }) => {
                    writer.write(format!("{},{},{},{}\n", robot_id, parcel, x, y).as_bytes())?;
                },
                (_, Vertex { x, y }) => {
                    writer.write(format!("{},{},{},{}\n", robot_id, -1, x, y).as_bytes())?;
                },
            }
        }
        writer.write("###\n".as_bytes())?;
        writer.flush()
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct RobotState {
    pub robot_id: usize,
    pub parcel_id: Option<usize>,
    pub vertex: Vertex,
}

#[derive(Debug, PartialEq)]
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
        self.states.len()
    }
}
