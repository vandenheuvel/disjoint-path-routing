use algorithm::path::PathAlgorithm;
use simulation::settings::Settings;
use algorithm::assignment::AssignmentAlgorithm;
use algorithm::NoSolutionError;
use simulation::state::History;
use simulation::Instructions;

pub struct ILPSteps<'p, 's, 'a> {
    settings: &'s Settings,
    assignment_algorithm: Box<AssignmentAlgorithm<'p, 's> + 'a>,
}

impl<'p, 's, 'a> PathAlgorithm<'p, 's, 'a> for ILPSteps {
    fn initialize(&mut self) -> Result<(), NoSolutionError> {
        unimplemented!()
    }

    fn next_step(&mut self, history: &History) -> Instructions {
        unimplemented!()
    }
}
