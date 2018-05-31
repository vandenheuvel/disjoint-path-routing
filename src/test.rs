use algorithm::greedy_shortest_paths::GreedyShortestPaths;
use algorithm::Algorithm;
use simulation::demand::uniform::Uniform;
use simulation::demand::Demand;
use simulation::plan::OneThreeRectangle;
use simulation::settings::Settings;
use simulation::simulation::Simulation;

#[test]
fn it_works() {
    let plan = OneThreeRectangle::new(3, 3);
    let settings = Settings {
        total_time: 15,
        maximum_robots: 2,
        nr_requests: 2,
    };
    let algorithm = Box::new(<GreedyShortestPaths as Algorithm>::instantiate(
        &plan, &settings,
    ));
    let demand = Box::new(<Uniform as Demand>::create(&[1, 2, 3]));

    let mut simulation = Simulation::new(algorithm, &plan, demand, &settings);
    simulation.initialize();
    simulation.run();
}
