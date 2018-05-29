use rand::{Rng, StdRng, SeedableRng};

use simulation::plan::Vertex;
use simulation::plan::Plan;

pub trait Demand {
    fn create(seed: &[usize]) -> Self where Self: Sized;
    fn generate(&mut self, plan: &Plan, nr_requests: u64) -> Vec<Request>;
}

#[derive(Copy, Clone)]
pub struct Request {
    pub id: u64,
    pub source: Vertex,
    pub terminal: Vertex,
}

/// Uniformly distributes source and demands
pub struct Uniform {
    rng: StdRng,
}

impl Demand for Uniform {
    fn create(seed: &[usize]) -> Uniform {
        Uniform {
            rng: StdRng::from_seed(seed),
        }
    }
    fn generate(&mut self, plan: &Plan, nr_requests: u64) -> Vec<Request> {
        let mut requests = Vec::new();

        let sources = plan.sources();
        let terminals = plan.terminals();

        for id in 0..nr_requests {
            let source = *self.rng.choose(&sources).unwrap();
            let terminal = *self.rng.choose(&terminals).unwrap();

            requests.push(Request { id, source, terminal, });
        }

        requests
    }
}
