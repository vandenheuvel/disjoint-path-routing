use algorithm::assignment::greedy_makespan::GreedyMakespan;
use algorithm::path::PathAlgorithm;
use algorithm::path::greedy_shortest_paths::GreedyShortestPaths;
use simulation::demand::uniform::Uniform;
use simulation::demand::Demand;
use simulation::plan::one_three_rectangle::OneThreeRectangle;
use simulation::plan::middle_terminals::MiddleTerminals;
use simulation::settings::Settings;
use simulation::simulation::Simulation;

macro_rules! simulate {
    (
        $test_name:ident,
        $plan:expr,
        $assignment:ident,
        $path:ident,
        $settings:expr,
        $seed:expr,
    ) => {
        #[test]
        fn $test_name() {
            let plan = $plan;
            let settings = $settings;
            let assignment_algorithm = $assignment::new(&plan, &settings);
            let path_algorithm = Box::new(<$path as PathAlgorithm>::instantiate(
                &plan,
                &settings,
                assignment_algorithm,
            ));
            let demand = Box::new(<Uniform as Demand>::create($seed));

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
    };
}

simulate!(
    greedy_makespan_greedy_shortest_paths,
    OneThreeRectangle::new(3, 3),
    GreedyMakespan,
    GreedyShortestPaths,
    Settings {
        total_time: 5,
        maximum_robots: 1,
        nr_requests: 1,
        output_file: None,
    },
    [0; 32],
);

simulate!(
    greedy_makespan_greedy_shortest_paths_long_time,
    OneThreeRectangle::new(3, 3),
    GreedyMakespan,
    GreedyShortestPaths,
    Settings {
        total_time: 60,
        maximum_robots: 1,
        nr_requests: 1,
        output_file: None,
    },
    [0; 32],
);

simulate!(
    greedy_makespan_greedy_shortest_paths_multiple_robots,
    OneThreeRectangle::new(30, 30),
    GreedyMakespan,
    GreedyShortestPaths,
    Settings {
        total_time: 1000,
        maximum_robots: 30,
        nr_requests: 300,
        output_file: None,
    },
    [0; 32],
);

simulate!(
    holes_greedy_makespan_greedy_shortest_paths_multiple_robots,
    MiddleTerminals::new(31, 31, 3, 4),
    GreedyMakespan,
    GreedyShortestPaths,
    Settings {
        total_time: 300,
        maximum_robots: 60,
        nr_requests: 600,
        output_file: None,
    },
    [0; 32],
);
