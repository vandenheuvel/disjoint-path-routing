use simulation::plan::Vertex;

pub mod demand;
pub mod plan;
pub mod settings;
pub mod simulation;
pub mod state;
pub mod statistics;


pub struct Instructions {
    pub movements: Vec<MoveInstruction>,
    pub placements: Vec<PlacementInstruction>,
    pub removals: Vec<RemovalInstruction>,
}
#[derive(Debug)]
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
