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
            self.settings.write(writer);
            self.plan.write(writer);
        }
    }
    fn setup_output(&mut self, output_file_name: &String) -> io::Result<()> {
        let file = OpenOptions::new().write(true).open(output_file_name)?;
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
    pub fn run(mut self) -> Result<History, Box<IllegalInstructionError>> {
        while self.history.last_state().requests.len() > 0
            && self.history.time() <= self.settings.total_time
        {
            let instructions = self.algorithm.next_step(&self.history);
            self.new_state(instructions)?;
            if let Some(ref mut writer) = self.output_writer {
                self.history.last_state().write(writer);
            }
        }

        Ok(self.history)
    }
    fn new_state(
        &mut self,
        instructions: Instructions,
    ) -> Result<(), Box<IllegalInstructionError>> {
        let mut used_vertices = self
            .history
            .last_state()
            .robot_states
            .iter()
            .filter(|robot| robot.vertex.is_some())
            .map(|robot| (robot.vertex.unwrap(), robot.robot_id))
            .collect::<HashMap<_, _>>();
        let mut newly_used_vertices = HashSet::new();

        let mut new_states = self.history.last_state().robot_states.clone();
        let mut new_requests = self.history.last_state().requests.clone();

        self.process_move_instructions(
            instructions.movements,
            &mut new_states,
            &used_vertices,
            &mut newly_used_vertices,
        ).map_err(|e| Box::new(e) as Box<IllegalInstructionError>)?;
        self.process_placement_instructions(
            instructions.placements,
            &mut new_states,
            &used_vertices,
            &mut newly_used_vertices,
            &mut new_requests,
        ).map_err(|e| Box::new(e) as Box<IllegalInstructionError>)?;
        self.process_removal_instructions(
            instructions.removals,
            &mut new_states,
            &mut new_requests,
            &mut newly_used_vertices,
        ).map_err(|e| Box::new(e) as Box<IllegalInstructionError>)?;

        Ok(self.history.states.push(State {
            robot_states: new_states,
            requests: new_requests,
        }))
    }
    fn process_move_instructions(
        &self,
        move_instructions: Vec<MoveInstruction>,
        new_states: &mut Vec<RobotState>,
        used_vertices: &HashMap<Vertex, usize>,
        newly_used_vertices: &mut HashSet<Vertex>,
    ) -> Result<(), IllegalMoveError> {
        for instruction in move_instructions {
            if let Some(error) = self.check_for_move_instruction_error(
                instruction,
                used_vertices,
                newly_used_vertices,
            ) {
                return Err(error);
            }
            let MoveInstruction { robot_id, vertex } = instruction;

            new_states[robot_id] = RobotState {
                robot_id,
                parcel_id: self.history.last_robot_state(robot_id).parcel_id,
                vertex: Some(vertex),
            };
            newly_used_vertices.insert(vertex);
            if let Some(previous_vertex) = self.history.last_robot_state(robot_id).vertex {
                newly_used_vertices.insert(previous_vertex);
            }
        }

        Ok(())
    }
    fn check_for_move_instruction_error(
        &self,
        instruction: MoveInstruction,
        used_vertices: &HashMap<Vertex, usize>,
        newly_used_vertices: &HashSet<Vertex>,
    ) -> Option<IllegalMoveError> {
        let MoveInstruction { robot_id, vertex } = instruction;

        match used_vertices.get(&vertex) {
            None => (),
            Some(robot) if *robot != instruction.robot_id => {
                return Some(IllegalMoveError::from(
                    instruction,
                    "State used in previous time step".to_string(),
                ));
            }
            _ => (),
        }
        if newly_used_vertices.contains(&vertex) {
            return Some(IllegalMoveError::from(
                instruction,
                "State will be used in next time step".to_string(),
            ));
        }
        match self.history.last_robot_state(robot_id).vertex {
            None => {
                return Some(IllegalMoveError {
                    instruction,
                    message: "Not yet placed on any vertex".to_string(),
                })
            }
            Some(previous_vertex) => {
                if previous_vertex.distance(vertex) > 1 {
                    return Some(IllegalMoveError {
                        instruction,
                        message: "Only one move at a time".to_string(),
                    });
                }
            }
        }

        None
    }
    fn process_placement_instructions(
        &self,
        placement_instructions: Vec<PlacementInstruction>,
        new_states: &mut Vec<RobotState>,
        used_vertices: &HashMap<Vertex, usize>,
        newly_used_vertices: &mut HashSet<Vertex>,
        new_requests: &mut HashMap<usize, Request>,
    ) -> Result<(), IllegalPlacementError> {
        for instruction in placement_instructions {
            if let Some(error) = self.check_for_placement_instruction_error(
                instruction,
                used_vertices,
                newly_used_vertices,
                new_requests,
            ) {
                return Err(error);
            }
            let ParcelInstruction {
                robot_id,
                parcel,
                vertex,
            } = instruction;
            new_states[robot_id] = RobotState {
                robot_id,
                parcel_id: Some(parcel),
                vertex: Some(vertex),
            };

            newly_used_vertices.insert(vertex);
        }

        Ok(())
    }
    fn check_for_placement_instruction_error(
        &self,
        instruction: PlacementInstruction,
        used_vertices: &HashMap<Vertex, usize>,
        newly_used_vertices: &mut HashSet<Vertex>,
        new_requests: &mut HashMap<usize, Request>,
    ) -> Option<IllegalPlacementError> {
        let PlacementInstruction {
            robot_id,
            parcel,
            vertex,
        } = instruction;

        if used_vertices.contains_key(&vertex) {
            return Some(IllegalPlacementError::from(
                instruction,
                "Vertex used in previous time step".to_string(),
            ));
        }
        if newly_used_vertices.contains(&vertex) {
            return Some(IllegalPlacementError::from(
                instruction,
                "Vertex already used in next time step".to_string(),
            ));
        }
        if self.history.last_robot_state(robot_id).vertex.is_some() {
            return Some(IllegalPlacementError::from(
                instruction,
                "Robot already placed".to_string(),
            ));
        }
        if !new_requests.contains_key(&parcel) {
            return Some(IllegalPlacementError::from(
                instruction,
                "Parcel no longer needed".to_string(),
            ));
        }

        None
    }
    fn process_removal_instructions(
        &self,
        removal_instructions: Vec<RemovalInstruction>,
        new_states: &mut Vec<RobotState>,
        new_requests: &mut HashMap<usize, Request>,
        used_vertices: &mut HashSet<Vertex>,
    ) -> Result<(), IllegalRemovalError> {
        for instruction in removal_instructions {
            if let Some(error) = self.check_for_removal_instruction_error(instruction) {
                return Err(error);
            }

            let RemovalInstruction {
                robot_id,
                parcel,
                vertex: _,
            } = instruction;
            new_states[robot_id] = RobotState {
                robot_id,
                parcel_id: None,
                vertex: None,
            };
            new_requests.remove(&parcel);

            if let Some(previous_vertex) = self.history.last_robot_state(robot_id).vertex {
                used_vertices.insert(previous_vertex);
            }
        }

        Ok(())
    }
    fn check_for_removal_instruction_error(
        &self,
        instruction: RemovalInstruction,
    ) -> Option<IllegalRemovalError> {
        match self.history.last_robot_state(instruction.robot_id).vertex {
            Some(vertex) => {
                if vertex != instruction.vertex {
                    return Some(IllegalRemovalError::from(
                        instruction,
                        "Robot is not at this location".to_string(),
                    ));
                }
            }
            None => {
                return Some(IllegalRemovalError::from(
                    instruction,
                    "Robot isn't placed".to_string(),
                ))
            }
        }

        match self
            .history
            .last_robot_state(instruction.robot_id)
            .parcel_id
        {
            None => {
                return Some(IllegalRemovalError::from(
                    instruction,
                    "Robot has no parcel".to_string(),
                ))
            }
            Some(parcel) if parcel != instruction.parcel => {
                return Some(IllegalRemovalError::from(
                    instruction,
                    "Robot holds other parcel".to_string(),
                ));
            }
            _ => (),
        }

        None
    }
}

pub struct Instructions {
    pub movements: Vec<MoveInstruction>,
    pub placements: Vec<PlacementInstruction>,
    pub removals: Vec<RemovalInstruction>,
}
pub enum Instruction {
    Move(MoveInstruction),
    Place(PlacementInstruction),
    Remove(RemovalInstruction),
}

#[derive(Debug, Copy, Clone)]
pub struct MoveInstruction {
    pub robot_id: usize,
    pub vertex: Vertex,
}
#[derive(Debug, Copy, Clone)]
pub struct ParcelInstruction {
    pub robot_id: usize,
    pub parcel: usize,
    pub vertex: Vertex,
}
pub type PlacementInstruction = ParcelInstruction;
pub type RemovalInstruction = ParcelInstruction;

pub trait IllegalInstructionError {
    fn instruction(&self) -> Instruction;
    fn message(&self) -> &String;
}
#[derive(Debug)]
pub struct IllegalMoveError {
    instruction: MoveInstruction,
    message: String,
}
impl IllegalMoveError {
    fn from(instruction: MoveInstruction, message: String) -> IllegalMoveError {
        IllegalMoveError {
            instruction,
            message,
        }
    }
}
impl IllegalInstructionError for IllegalMoveError {
    fn instruction(&self) -> Instruction {
        Instruction::Move(self.instruction)
    }
    fn message(&self) -> &String {
        &self.message
    }
}

pub struct IllegalPlacementError {
    instruction: PlacementInstruction,
    message: String,
}
impl IllegalPlacementError {
    fn from(instruction: PlacementInstruction, message: String) -> IllegalPlacementError {
        IllegalPlacementError {
            instruction,
            message,
        }
    }
}
impl IllegalInstructionError for IllegalPlacementError {
    fn instruction(&self) -> Instruction {
        Instruction::Place(self.instruction)
    }
    fn message(&self) -> &String {
        &self.message
    }
}

pub struct IllegalRemovalError {
    instruction: RemovalInstruction,
    message: String,
}
impl IllegalRemovalError {
    fn from(instruction: RemovalInstruction, message: String) -> IllegalRemovalError {
        IllegalRemovalError {
            instruction,
            message,
        }
    }
}
impl IllegalInstructionError for IllegalRemovalError {
    fn instruction(&self) -> Instruction {
        Instruction::Remove(self.instruction)
    }
    fn message(&self) -> &String {
        &self.message
    }
}
