use simulation::plan::Plan;
use simulation::plan::Vertex;

pub mod uniform;

pub trait Demand {
    fn create(seed: [u8; 32]) -> Self
    where
        Self: Sized;
    fn generate(&mut self, plan: &Plan, nr_requests: u64) -> Vec<Request>;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Request {
    pub from: Vertex,
    pub to: Vertex,
}

impl Request {
    pub fn distance(&self) -> u64 {
        self.from.distance(self.to)
    }
}
