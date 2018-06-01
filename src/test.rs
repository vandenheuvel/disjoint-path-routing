use algorithm::greedy_shortest_paths::GreedyShortestPaths;
use algorithm::Algorithm;
use simulation::demand::uniform::Uniform;
use simulation::demand::Demand;
use simulation::plan::one_three_rectangle::OneThreeRectangle;
use simulation::settings::Settings;
use simulation::simulation::Simulation;

#[test]
fn it_works() {
    let plan = OneThreeRectangle::new(5, 5);
    let settings = Settings {
        total_time: 15,
        maximum_robots: 2,
        nr_requests: 2,
        real_time: false,
        output_file: None,
    };
    let algorithm = Box::new(<GreedyShortestPaths as Algorithm>::instantiate(
        &plan, &settings,
    ));
    let demand = Box::new(<Uniform as Demand>::create(&[1, 2, 3]));

    let mut simulation = Simulation::new(algorithm, &plan, demand, &settings);
    simulation.initialize();
    simulation.run();
}
