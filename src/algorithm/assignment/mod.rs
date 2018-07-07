use fnv::FnvHashMap;
use simulation::demand::Request;
use simulation::plan::Vertex;

pub mod greedy_makespan;
pub mod makespan_single_vehicle_ilp;
pub mod multiple_vehicle_ilp;

const RUN_FILE_NAME: &str = "run.run";
const DAT_FILE_NAME: &str = "data.dat";

pub trait AssignmentAlgorithm<'p, 's> {
    fn calculate_assignment(
        &mut self,
        requests: &FnvHashMap<usize, Request>,
        availability: &Vec<(usize, Vertex)>,
    ) -> Vec<Vec<usize>>;
}

struct LPIOError {}

#[cfg(test)]
pub mod test {

    use simulation::plan::one_three_rectangle::OneThreeRectangle;
    use simulation::plan::Plan;
    use simulation::settings::Settings;

    pub fn get_test_variables(robots: usize, requests: u64) -> (impl Plan, Settings) {
        let plan = OneThreeRectangle::new(3, 3);
        let settings = Settings {
            total_time: 5,
            nr_robots: robots,
            nr_requests: requests,
            output_file: None,
        };
        (plan, settings)
    }
}
