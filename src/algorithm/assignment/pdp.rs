use algorithm::assignment::AssignmentAlgorithm;
use fnv::FnvHashMap;
use lp_modeler::operations::LpOperations;
use lp_modeler::problem::{LpObjective, LpProblem};
use lp_modeler::solvers::{GurobiSolver, SolverTrait};
use lp_modeler::variables::LpInteger;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::settings::Settings;

//pub struct OptimalIlpFormulation<'p, 's> {
//    plan: &'p (Plan + 'p),
//    settings: &'s Settings,
//}
//
//impl<'p, 's> AssignmentAlgorithm<'p, 's> for OptimalIlpFormulation<'p, 's> {
//    fn instantiate(plan: &'p impl Plan, settings: &'s Settings) -> Self where Self: Sized {
//        OptimalIlpFormulation {
//            plan,
//            settings,
//        }
//    }
//    fn calculate_assignment(&mut self, requests: &FnvHashMap<usize, Request>) -> Vec<Vec<Request>> {
//
//    }
//}

fn that() {}

fn this() {
    let ref a = LpInteger::new("a");
    let ref b = LpInteger::new("b");
    let ref c = LpInteger::new("c");

    let mut problem = LpProblem::new("One Problem", LpObjective::Maximize);

    // Maximize 10*a + 20*b
    problem += 10.0 * a + 20.0 * b;

    // 500*a + 1200*b + 1500*c <= 10000
    problem += (500 * a + 1200 * b + 1500 * c).le(10000);
    // a <= b
    problem += (a).le(b);

    let solver = GurobiSolver::new();
    let solver = solver.command_name("/opt/gurobi800/linux64/bin/gurobi_cl".to_string());

    match solver.run(&problem) {
        Ok((status, res)) => {
            println!("Status {:?}", status);
            for (name, value) in res.iter() {
                println!("value of {} = {}", name, value);
            }
        }
        Err(msg) => println!("{}", msg),
    }
}

#[cfg(test)]
mod test {

    use super::*;

    #[test]
    fn test() {
        this();
    }

}
