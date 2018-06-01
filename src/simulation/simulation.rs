use algorithm::Algorithm;
use simulation::demand::Demand;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::plan::Vertex;
use simulation::settings::Settings;
use simulation::state::History;
use simulation::state::RobotState;
use simulation::state::State;
use std::collections::HashMap;
use std::collections::HashSet;
use std::fs::File;
use std::fs::OpenOptions;
use std::io;
use std::io::BufWriter;
use std::io::Write;

pub struct Simulation<'a, 'p, 's> {
    algorithm: Box<Algorithm<'p, 's> + 'a>,
    plan: &'p Plan,
    demand: Box<Demand + 'a>,
    settings: &'s Settings,

    history: History,
    output_writer: Option<BufWriter<File>>,
}

impl<'a, 'p, 's> Simulation<'a, 'p, 's> {
    pub fn new(
        algorithm: Box<Algorithm<'p, 's> + 'a>,
        plan: &'p Plan,
        demand: Box<Demand + 'a>,
        settings: &'s Settings,
    ) -> Simulation<'a, 'p, 's> {
        Simulation {
            algorithm,
            plan,
            demand,
            settings,

            history: History::empty(),
            output_writer: None,
        }
    }
    /// Gets the initial state of the system set up, creates history of time 0.
    pub fn initialize(&mut self) {
        if let Some(ref file_name) = self.settings.output_file {
            self.setup_output(file_name).unwrap();
        }
        self.set_initial_state();
        self.algorithm
            .initialize(&self.history.last_state().requests);

        if let Some(ref mut writer) = self.output_writer {
            self.plan.write(writer)
        }
    }
    fn setup_output(&mut self, output_file_name: &String) -> io::Result<()> {
        let mut file = OpenOptions::new().write(true).open(output_file_name)?;
        let buffered_writer = BufWriter::new(file);

        Ok(self.output_writer = Some(buffered_writer))
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
        let requests = self
            .demand
            .generate(self.plan, self.settings.nr_requests)
            .into_iter()
            .enumerate()
            .collect();

        self.history.states.push(State {
            robot_states,
            requests,
        });
    }
    pub fn run(&mut self) {
        while self.history.last_state().requests.len() > 0
            && self.history.time() <= self.settings.total_time {
            let instructions = self.algorithm.next_step(&self.history);
            self.new_state(instructions);
            if let Some(ref mut writer) = self.output_writer {
                self.history.last_state().write(writer);
            }
        }

        println!("{:?}", self.history.time());
    }
    fn new_state(&mut self, instructions: Instructions) -> Result<(), String> {
        let mut used_states = HashSet::new();
        let mut new_states = self.history.last_state().robot_states.clone();
        let mut new_requests = self.history.last_state().requests.clone();

        self.process_move_instructions(instructions.movements, &mut new_states, &mut used_states);
        self.process_placement_instructions(
            instructions.placements,
            &mut new_states,
            &mut used_states,
        );
        self.process_removal_instructions(
            instructions.removals,
            &mut new_states,
            &mut new_requests,
            &mut used_states,
        );

        Ok(self.history.states.push(State {
            robot_states: new_states,
            requests: new_requests,
        }))
    }
    fn process_move_instructions(
        &self,
        move_instructions: Vec<MoveInstruction>,
        new_states: &mut Vec<RobotState>,
        used_states: &mut HashSet<Vertex>,
    ) {
        for MoveInstruction {
            robot_id,
            parcel,
            vertex,
        } in move_instructions
        {
            if !used_states.contains(&vertex) {
                new_states[robot_id] = RobotState {
                    robot_id,
                    parcel_id: parcel,
                    vertex: Some(vertex),
                };
            }

            used_states.insert(vertex);
            if let Some(previous_vertex) = self.history.last_robot_state(robot_id).vertex {
                used_states.insert(previous_vertex);
            }
        }
    }
    fn process_placement_instructions(
        &self,
        placement_instructions: Vec<PlacementInstruction>,
        new_states: &mut Vec<RobotState>,
        used_states: &mut HashSet<Vertex>,
    ) {
        for PlacementInstruction {
            robot_id,
            parcel,
            vertex,
        } in placement_instructions
        {
            if !used_states.contains(&vertex) {
                new_states[robot_id] = RobotState {
                    robot_id,
                    parcel_id: Some(parcel),
                    vertex: Some(vertex),
                };
            }

            used_states.insert(vertex);
            if let Some(previous_vertex) = self.history.last_robot_state(robot_id).vertex {
                used_states.insert(previous_vertex);
            }
        }
    }
    fn process_removal_instructions(
        &self,
        removal_instructions: Vec<RemovalInstruction>,
        new_states: &mut Vec<RobotState>,
        new_requests: &mut HashMap<usize, Request>,
        used_states: &mut HashSet<Vertex>,
    ) {
        for RemovalInstruction {
            robot_id,
            parcel,
            vertex,
        } in removal_instructions
        {
            if !used_states.contains(&vertex) {
                new_states[robot_id] = RobotState {
                    robot_id,
                    parcel_id: None,
                    vertex: None,
                };
            }
            new_requests.remove(&parcel);

            if let Some(previous_vertex) = self.history.last_robot_state(robot_id).vertex {
                used_states.insert(previous_vertex);
            }
        }
    }
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
