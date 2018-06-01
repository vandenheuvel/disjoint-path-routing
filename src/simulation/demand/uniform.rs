use rand::{Rng, SeedableRng, StdRng};
use simulation::demand::Demand;
use simulation::demand::Request;
use simulation::plan::Plan;

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

        for _ in 0..nr_requests {
            let source = *self.rng.choose(&sources).unwrap();
            let terminal = *self.rng.choose(&terminals).unwrap();

            requests.push(Request { source, terminal });
        }

        requests
    }
}