use simulation::simulation::Simulation;
use simulation::settings::Settings;
use simulation::plan::OneThreeRectangle;
use algorithm::greedy_shortest_paths::GreedyShortestPaths;
use simulation::demand::Uniform;
use simulation::demand::Demand;
use algorithm::Algorithm;

#[test]
fn it_works() {
    let plan = OneThreeRectangle::new(3, 4);
    let settings = Settings::default();
    let algorithm = Box::new(<GreedyShortestPaths as Algorithm>::instantiate(&plan, &settings));
    let demand = Box::new(<Uniform as Demand>::create(&[1, 2, 3]));

    let mut simulation = Simulation::new(algorithm, &plan, demand, &settings);
    simulation.initialize();
    let results = simulation.run();
}
