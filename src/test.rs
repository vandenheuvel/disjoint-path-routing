use algorithm::greedy_shortest_paths::GreedyShortestPaths;
use algorithm::Algorithm;
use simulation::demand::uniform::Uniform;
use simulation::demand::Demand;
use simulation::plan::one_three_rectangle::OneThreeRectangle;
use simulation::settings::Settings;
use simulation::simulation::Simulation;

#[test]
fn it_works() {
    let plan = OneThreeRectangle::new(25, 25);
    let settings = Settings {
        total_time: 200,
        maximum_robots: 20,
        nr_requests: 60,
        real_time: false,
        //        output_file: Some("/tmp/disjoint".to_string()),
        output_file: None,
    };
    let algorithm = Box::new(<GreedyShortestPaths as Algorithm>::instantiate(
        &plan, &settings,
    ));
    let demand = Box::new(<Uniform as Demand>::create(&[1, 2, 3]));

    let mut simulation = Simulation::new(algorithm, &plan, demand, &settings);
    simulation.initialize();
    if let Err(error) = simulation.run() {
        println!("{:?}: {:?}", error.message(), error.instruction());
    };
}
