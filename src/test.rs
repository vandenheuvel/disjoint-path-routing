use algorithm::greedy_shortest_paths::GreedyShortestPaths;
use algorithm::Algorithm;
use simulation::demand::uniform::Uniform;
use simulation::demand::Demand;
use simulation::plan::one_three_rectangle::OneThreeRectangle;
use simulation::settings::Settings;
use simulation::simulation::Simulation;

#[test]
fn integration() {
    let plan = OneThreeRectangle::new(3, 3);
    let settings = Settings {
        total_time: 5,
        maximum_robots: 1,
        nr_requests: 1,
        real_time: false,
        output_file: None,
    };
    let algorithm = Box::new(<GreedyShortestPaths as Algorithm>::instantiate(
        &plan, &settings,
    ));
    let demand = Box::new(<Uniform as Demand>::create(&[1, 3]));

    let mut simulation = Simulation::new(algorithm, &plan, demand, &settings);
    simulation.initialize().ok().unwrap();
    let result = simulation.run();


    let is_ok = result.is_ok();
    if let Err(error) = result {
        println!("{:?}: {:?} at time {}", error.message(), error.instruction(), error.time());
    };
    assert!(is_ok);
}
