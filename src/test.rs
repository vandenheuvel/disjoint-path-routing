use algorithm::assignment::greedy_makespan::GreedyMakespan;
use algorithm::assignment::multiple_vehicle_ilp::MultiVehicleIlpFormulation;
use algorithm::path::greedy_shortest_paths::GreedyShortestPaths;
use algorithm::path::ilp::ILPSteps;
use simulation::demand::uniform::Uniform;
use simulation::demand::Demand;
use simulation::plan::e_plan::EPlan;
use simulation::plan::middle_terminals::MiddleTerminals;
use simulation::plan::one_three_rectangle::OneThreeRectangle;
use simulation::settings::Settings;
use simulation::simulation::Simulation;

#[test]
fn multiple_vehicle_ilp_path() {
    let plan = OneThreeRectangle::new(3, 3);
    let settings = Settings {
        total_time: 5,
        nr_robots: 1,
        nr_requests: 1,
        output_file: None,
    };
    let assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
    let path_algorithm = Box::new(GreedyShortestPaths::new(
        &plan,
        &settings,
        assignment_algorithm,
    ));
    let demand = Box::new(<Uniform as Demand>::create([0; 32]));

    let mut simulation = Simulation::new(path_algorithm, &plan, demand, &settings);
    simulation.initialize().ok().unwrap();
    let result = simulation.run();

    let is_ok = result.is_ok();
    if let Err(error) = result {
        println!(
            "{:?}: {:?} at time {}",
            error.message(),
            error.instruction(),
            error.time()
        );
    };
    assert!(is_ok);
}

#[test]
fn multiple_vehicle_ilp_path_large() {
    let plan = OneThreeRectangle::new(30, 30);
    let settings = Settings {
        total_time: 500,
        nr_robots: 5,
        nr_requests: 10,
        //        output_file: Some("/tmp/disjoint".to_string()),
        output_file: None,
    };
    let assignment_algorithm = Box::new(MultiVehicleIlpFormulation::new(&plan, &settings));
    let path_algorithm = Box::new(ILPSteps::new(&plan, &settings, assignment_algorithm, 2));
    let demand = Box::new(<Uniform as Demand>::create([0; 32]));

    let mut simulation = Simulation::new(path_algorithm, &plan, demand, &settings);
    simulation.initialize().ok().unwrap();
    let result = simulation.run();

    let is_ok = result.is_ok();
    if let Err(error) = result {
        println!(
            "{:?}: {:?} at time {}",
            error.message(),
            error.instruction(),
            error.time()
        );
    };
    assert!(is_ok);
}

#[test]
fn e_plan_multiple_vehicle_ilp_path_large() {
    let plan = EPlan::new(62, 63, 20, 3);
    let settings = Settings {
        total_time: 500,
        nr_robots: 5,
        nr_requests: 10,
        output_file: Some("/tmp/disjoint".to_string()),
        //        output_file: None,
    };
    let assignment_algorithm = Box::new(MultiVehicleIlpFormulation::new(&plan, &settings));
    let path_algorithm = Box::new(ILPSteps::new(&plan, &settings, assignment_algorithm, 2));
    let demand = Box::new(<Uniform as Demand>::create([0; 32]));

    let mut simulation = Simulation::new(path_algorithm, &plan, demand, &settings);
    simulation.initialize().ok().unwrap();
    let result = simulation.run();

    let is_ok = result.is_ok();
    if let Err(error) = result {
        println!(
            "{:?}: {:?} at time {}",
            error.message(),
            error.instruction(),
            error.time()
        );
    };
    assert!(is_ok);
}

#[test]
fn e_plan_greedy() {
    let plan = MiddleTerminals::new(10, 10, 3, 3);
    let settings = Settings {
        total_time: 500,
        nr_robots: 5,
        nr_requests: 10,
        //        output_file: Some("/tmp/disjoint".to_string()),
        output_file: None,
    };
    let assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
    let path_algorithm = Box::new(GreedyShortestPaths::new(
        &plan,
        &settings,
        assignment_algorithm,
    ));
    let demand = Box::new(<Uniform as Demand>::create([0; 32]));

    let mut simulation = Simulation::new(path_algorithm, &plan, demand, &settings);
    simulation.initialize().ok().unwrap();
    let result = simulation.run();

    let is_ok = result.is_ok();
    if let Err(error) = result {
        println!(
            "{:?}: {:?} at time {}",
            error.message(),
            error.instruction(),
            error.time()
        );
    };
    assert!(is_ok);
}
