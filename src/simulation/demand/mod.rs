use simulation::plan::Plan;
use simulation::plan::Vertex;

pub mod uniform;

pub trait Demand {
    fn create(seed: &[usize]) -> Self where Self: Sized;
    fn generate(&mut self, plan: &Plan, nr_requests: u64) -> Vec<Request>;
}

#[derive(Copy, Clone)]
pub struct Request {
    pub source: Vertex,
    pub terminal: Vertex,
}
