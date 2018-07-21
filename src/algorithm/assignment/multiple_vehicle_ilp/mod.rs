use fnv::FnvHashMap;
use itertools::Itertools;

use algorithm::assignment::greedy_makespan::GreedyMakespan;
use algorithm::assignment::AssignmentAlgorithm;
use algorithm::assignment::LPIOError;
use algorithm::DAT_FILE_NAME;
use algorithm::RUN_FILE_NAME;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::plan::Vertex;
use simulation::settings::Settings;
use std::env::temp_dir;
use std::fs::create_dir;
use std::fs::remove_dir_all;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::path::PathBuf;
use std::process::Command;

const MOD_FILE_PATH: &str = "/home/bram/git/disjoint-path-routing/src/algorithm/assignment/multiple_vehicle_ilp/multiple_vehicle_ilp.mod";
const WORKING_DIRECTORY: &str = "multiple_vehicle_ilp";

pub struct MultiVehicleIlpFormulation<'p, 's> {
    plan: &'p (Plan + 'p),
    settings: &'s Settings,
}

impl<'p, 's> MultiVehicleIlpFormulation<'p, 's> {
    pub fn new(plan: &'p impl Plan, settings: &'s Settings) -> MultiVehicleIlpFormulation<'p, 's> {
        MultiVehicleIlpFormulation { plan, settings }
    }
    fn get_paths() -> (PathBuf, PathBuf, PathBuf, PathBuf) {
        let model_path = <&str as AsRef<Path>>::as_ref(&MOD_FILE_PATH).to_path_buf();

        let working_directory = temp_dir().join(WORKING_DIRECTORY);
        let dat_path = working_directory.join(DAT_FILE_NAME);
        let run_path = working_directory.join(RUN_FILE_NAME);

        (model_path, working_directory, dat_path, run_path)
    }
    fn calculate_start_costs(
        availability: &Vec<(usize, Vertex)>,
        requests: &FnvHashMap<usize, Request>,
    ) -> Vec<(usize, usize, u64)> {
        let mut costs = Vec::with_capacity(availability.len() * requests.len());

        for (robot, &(time, current_location)) in availability.iter().enumerate() {
            for (&request_id, request) in requests.iter() {
                let start_distance = current_location.distance(request.from);
                costs.push((robot, request_id, start_distance + request.distance()));
            }
        }

        costs
    }
    fn calculate_transition_costs(
        requests: &FnvHashMap<usize, Request>,
    ) -> Vec<(usize, usize, u64)> {
        let mut costs = Vec::with_capacity(requests.len() * requests.len());

        for (&id1, request1) in requests.iter() {
            for (&id2, request2) in requests.iter() {
                let in_between_distance = request1.to.distance(request2.from);
                costs.push((id1, id2, in_between_distance + request2.distance()));
            }
        }

        costs
    }
    fn calculate_end_costs(
        availability: &Vec<(usize, Vertex)>,
        requests: &FnvHashMap<usize, Request>,
    ) -> Vec<(usize, usize, u64)> {
        let mut costs = Vec::with_capacity(availability.len() * requests.len());

        for &(robot, _) in availability.iter() {
            for (&request_id, _) in requests.iter() {
                costs.push((robot, request_id, 0));
            }
        }

        costs
    }
    fn write_data_file(
        path: impl AsRef<Path>,
        availability: &Vec<(usize, Vertex)>,
        requests: &FnvHashMap<usize, Request>,
        start_costs: Vec<(usize, usize, u64)>,
        transition_costs: Vec<(usize, usize, u64)>,
        end_costs: Vec<(usize, usize, u64)>,
        initial_solution: Vec<Vec<usize>>,
    ) -> Result<(), LPIOError> {
        let mut file = File::create(path).unwrap();

        file.write("set ROBOTS :=\n".as_ref());
        for (robot, (_, _)) in availability.iter().enumerate() {
            write!(file, "  {},\n", robot);
        }
        file.write(";\n".as_ref());

        file.write("set REQUESTS :=\n".as_ref());
        for &request in requests.keys() {
            write!(file, "  {},\n", request);
        }
        file.write(";\n".as_ref());

        file.write("param start_cost default 0 :=\n".as_ref());
        for (robot, request, cost) in start_costs {
            if cost > 0 {
                write!(file, "{} {} {}\n", robot, request, cost);
            }
        }
        file.write(";\n".as_ref());

        file.write("param transition_cost default 0 :=\n".as_ref());
        for (request_i, request_j, cost) in transition_costs {
            if cost > 0 {
                write!(file, "{} {} {}\n", request_i, request_j, cost);
            }
        }
        file.write(";\n".as_ref());

        file.write("param end_cost default 0 :=\n".as_ref());
        for (request, end_location, cost) in end_costs {
            if cost > 0 {
                write!(file, "{} {} {}\n", request, end_location, cost);
            }
        }
        file.write(";\n".as_ref());

        writeln!(file, "var First_Request :=");
        for (robot, requests) in initial_solution.iter().enumerate() {
            writeln!(file, "{} {} {}", robot, requests[0], 1);
        }
        writeln!(file, ";");

        writeln!(file, "var Transition :=");
        for (robot, requests) in initial_solution.iter().enumerate() {
            for i in 0..(requests.len() - 1) {
                writeln!(file, "{} {} {} {}", robot, requests[i], requests[i + 1], 1);
            }
        }
        writeln!(file, ";");

        writeln!(file, "var Last_Request :=");
        for (robot, requests) in initial_solution.iter().enumerate() {
            writeln!(
                file,
                "{} {} {} {}",
                robot,
                robot,
                requests.last().unwrap(),
                1
            );
        }
        writeln!(file, ";");

        Ok(())
    }
    fn write_run_file<T>(path: T, model_path: T, data_path: T, time_limit: f64)
    where
        T: AsRef<Path>,
    {
        let mut file = File::create(path).unwrap();

        writeln!(file, "model '{}';", model_path.as_ref().to_str().unwrap());
        writeln!(file, "data '{}';", data_path.as_ref().to_str().unwrap());
        file.write("option solver '/home/bram/Downloads/amplide.linux64/gurobi';\n".as_ref());
        file.write("option show_stats 0;\n".as_ref());
        writeln!(file, "option gurobi_options 'timelim {} bestbound 1';", 30);
        file.write("solve;\n".as_ref());
        file.write("option omit_zero_rows 1;\n".as_ref());
        file.write("option display_1col 1000000;\n".as_ref());
        file.write("display First_Request;\n".as_ref());
        file.write("display Transition;\n".as_ref());
        file.write("display Last_Request;\n".as_ref());
        file.write("option omit_zero_rows 0;\n".as_ref());
        file.write("display Robot_Number;\n".as_ref());
    }
    fn calculate_time_limit(
        availability: &Vec<(usize, Vertex)>,
        requests: &FnvHashMap<usize, Request>,
    ) -> f64 {
        requests.iter().map(|(_, r)| r.distance()).sum::<u64>() as f64 / availability.len() as f64
    }
    fn parse_ampl_output(
        output: Vec<u8>,
    ) -> (
        FnvHashMap<usize, usize>,
        FnvHashMap<(usize, usize), usize>,
        FnvHashMap<(usize, usize), usize>,
        u64,
        f64,
    ) {
        let output = String::from_utf8(output).unwrap();
        let lines = output.lines().collect::<Vec<_>>();

        let objective = lines
            .iter()
            .skip_while(|line| !line.contains("objective"))
            .next()
            .unwrap()
            .split_whitespace()
            .last()
            .unwrap()
            .parse::<u64>()
            .unwrap();
        let relmipgap = if let Some(line) = lines
            .iter()
            .skip_while(|line| !line.contains("relmipgap"))
            .next()
        {
            line.split_whitespace()
                .last()
                .unwrap()
                .parse::<f64>()
                .unwrap()
        } else {
            0 as f64
        };

        let mut segments = lines.split(|&line| line == ";");
        let mut get_lines = |pattern: &str| {
            segments
                .next()
                .unwrap()
                .into_iter()
                .skip_while(|&&line| !line.starts_with(pattern))
                .skip(1)
                .map(|line| *line)
                .collect::<Vec<_>>()
        };
        let firsts = get_lines("First_Request");
        let transitions = get_lines("Transition");
        let lasts = get_lines("Last_Request");

        let first = MultiVehicleIlpFormulation::parse_first_requests(firsts);
        let transitions = MultiVehicleIlpFormulation::parse_transitions(transitions);
        let lasts = MultiVehicleIlpFormulation::parse_last_locations(lasts);

        (first, transitions, lasts, objective, relmipgap)
    }
    fn parse_first_requests(lines: Vec<&str>) -> FnvHashMap<usize, usize> {
        MultiVehicleIlpFormulation::parse_lines_2d(lines)
    }
    fn parse_lines_2d(lines: Vec<&str>) -> FnvHashMap<usize, usize> {
        lines
            .into_iter()
            .map(
                |line| match line.split_whitespace().collect_vec().as_slice() {
                    [first, second, "1"] => {
                        if let (Ok(first), Ok(second)) =
                            (first.parse::<usize>(), second.parse::<usize>())
                        {
                            (first, second)
                        } else {
                            panic!()
                        }
                    }
                    _ => panic!(),
                },
            )
            .collect()
    }
    fn parse_transitions(lines: Vec<&str>) -> FnvHashMap<(usize, usize), usize> {
        MultiVehicleIlpFormulation::parse_lines_3d(lines)
    }
    fn parse_last_locations(lines: Vec<&str>) -> FnvHashMap<(usize, usize), usize> {
        MultiVehicleIlpFormulation::parse_lines_3d(lines)
    }
    fn parse_lines_3d(lines: Vec<&str>) -> FnvHashMap<(usize, usize), usize> {
        lines
            .into_iter()
            .map(
                |line| match line.split_whitespace().collect_vec().as_slice() {
                    [first, second, third, "1"] => {
                        if let (Ok(first), Ok(second), Ok(third)) = (
                            first.parse::<usize>(),
                            second.parse::<usize>(),
                            third.parse::<usize>(),
                        ) {
                            (first, second, third)
                        } else {
                            panic!()
                        }
                    }
                    _ => panic!(),
                },
            )
            .map(|(first, second, third)| ((first, second), third))
            .collect()
    }
    fn reconstruct_assignment(
        firsts: FnvHashMap<usize, usize>,
        transitions: FnvHashMap<(usize, usize), usize>,
        lasts: FnvHashMap<(usize, usize), usize>,
    ) -> Vec<Vec<usize>> {
        let missing = (0..firsts.len())
            .filter(|i| !firsts.contains_key(i))
            .collect::<Vec<_>>();
        let mut assignments = FnvHashMap::default();

        for (robot, first_request) in firsts {
            assignments.insert(
                robot,
                MultiVehicleIlpFormulation::reconstruct_transitions(
                    robot,
                    first_request,
                    &transitions,
                ),
            );
        }
        for robot in missing {
            assignments.insert(robot, Vec::with_capacity(0));
        }

        let mut assignments = assignments.into_iter().collect::<Vec<_>>();
        assignments.sort_by_key(|&(robot, _)| robot);
        assignments
            .into_iter()
            .map(|(_, assigned)| assigned)
            .collect()
    }
    fn reconstruct_transitions(
        robot: usize,
        first_request: usize,
        transitions: &FnvHashMap<(usize, usize), usize>,
    ) -> Vec<usize> {
        let mut assigned = Vec::new();
        assigned.push(first_request);

        let mut current = first_request;
        while let Some(next) = transitions.get(&(robot, current)) {
            current = *next;
            assigned.push(current);
        }

        assigned
    }
    pub fn calculate_assignment_quality(
        &mut self,
        requests: &FnvHashMap<usize, Request>,
        availability: &Vec<(usize, Vertex)>,
    ) -> (u64, f64) {
        let (model_path, working_directory, data_path, run_path) =
            MultiVehicleIlpFormulation::get_paths();
        if working_directory.exists() {
            remove_dir_all(working_directory.as_path());
        }
        create_dir(working_directory).unwrap();

        let start_costs = MultiVehicleIlpFormulation::calculate_start_costs(availability, requests);
        let transition_costs = MultiVehicleIlpFormulation::calculate_transition_costs(requests);
        let end_costs = MultiVehicleIlpFormulation::calculate_end_costs(availability, requests);
        let mut pre_algorithm = GreedyMakespan::new(self.plan, self.settings);
        let initial_assignment = pre_algorithm.calculate_assignment(requests, availability);
        MultiVehicleIlpFormulation::write_data_file(
            data_path.as_path(),
            availability,
            requests,
            start_costs,
            transition_costs,
            end_costs,
            initial_assignment,
        );
        MultiVehicleIlpFormulation::write_run_file(
            run_path.as_path(),
            model_path.as_path(),
            data_path.as_path(),
            MultiVehicleIlpFormulation::calculate_time_limit(availability, requests),
        );

        let output = Command::new("/home/bram/Downloads/amplide.linux64/ampl")
            .arg(run_path)
            .output()
            .unwrap();

        let (_, _, _, objective, relmipgap) =
            MultiVehicleIlpFormulation::parse_ampl_output(output.stdout);
        (objective, relmipgap)
    }
}

impl<'p, 's> AssignmentAlgorithm<'p, 's> for MultiVehicleIlpFormulation<'p, 's> {
    fn calculate_assignment(
        &mut self,
        requests: &FnvHashMap<usize, Request>,
        availability: &Vec<(usize, Vertex)>,
    ) -> Vec<Vec<usize>> {
        let (model_path, working_directory, data_path, run_path) =
            MultiVehicleIlpFormulation::get_paths();
        if working_directory.exists() {
            remove_dir_all(working_directory.as_path());
        }
        create_dir(working_directory).unwrap();

        let start_costs = MultiVehicleIlpFormulation::calculate_start_costs(availability, requests);
        let transition_costs = MultiVehicleIlpFormulation::calculate_transition_costs(requests);
        let end_costs = MultiVehicleIlpFormulation::calculate_end_costs(availability, requests);
        let mut pre_algorithm = GreedyMakespan::new(self.plan, self.settings);
        let initial_assignment = pre_algorithm.calculate_assignment(requests, availability);
        if transition_costs.len() * availability.len() > 10_000_000 {
            panic!()
        }
        MultiVehicleIlpFormulation::write_data_file(
            data_path.as_path(),
            availability,
            requests,
            start_costs,
            transition_costs,
            end_costs,
            initial_assignment,
        );
        MultiVehicleIlpFormulation::write_run_file(
            run_path.as_path(),
            model_path.as_path(),
            data_path.as_path(),
            MultiVehicleIlpFormulation::calculate_time_limit(availability, requests),
        );

        let output = Command::new("/home/bram/Downloads/amplide.linux64/ampl")
            .arg(run_path)
            .output()
            .unwrap();

        let (firsts, transitions, lasts, objective, relmipgap) =
            MultiVehicleIlpFormulation::parse_ampl_output(output.stdout);
        MultiVehicleIlpFormulation::reconstruct_assignment(firsts, transitions, lasts)
    }
}

#[cfg(test)]
mod test {

    use super::*;
    use algorithm::assignment::test::get_test_variables;

    #[test]
    fn assignment_one() {
        let (plan, settings) = get_test_variables(1, 1);
        let mut algorithm = MultiVehicleIlpFormulation::new(&plan, &settings);

        let requests = map!
        [
            0 => Request {
                from: Vertex { x: 0, y: 0, },
                to: Vertex { x: 0, y: 1, },
            },
        ];
        let availability = vec![(0, Vertex { x: 0, y: 0 })];

        let result = algorithm.calculate_assignment(&requests, &availability);
        assert_eq!(result, vec![vec![0]]);
    }

    #[test]
    fn assignment_two() {
        let (plan, settings) = get_test_variables(1, 2);
        let mut algorithm = MultiVehicleIlpFormulation::new(&plan, &settings);

        let requests = map!
        [
            0 => Request {
                from: Vertex { x: 0, y: 0, },
                to: Vertex { x: 0, y: 1, },
            },
            1 => Request {
                from: Vertex { x: 0, y: 1, },
                to: Vertex { x: 0, y: 2, },
            },
        ];
        let availability = vec![(0, Vertex { x: 0, y: 0 })];

        let result = algorithm.calculate_assignment(&requests, &availability);
        assert_eq!(result, vec![vec![0, 1]]);
    }

    #[test]
    fn assignment_two_robots() {
        let (plan, settings) = get_test_variables(2, 2);
        let mut algorithm = MultiVehicleIlpFormulation::new(&plan, &settings);

        let requests = map!
        [
            0 => Request {
                from: Vertex { x: 0, y: 0, },
                to: Vertex { x: 0, y: 1, },
            },
            1 => Request {
                from: Vertex { x: 10, y: 10, },
                to: Vertex { x: 0, y: 2, },
            },
        ];
        let availability = vec![(0, Vertex { x: 0, y: 0 }), (1, Vertex { x: 10, y: 10 })];

        let result = algorithm.calculate_assignment(&requests, &availability);
        assert_eq!(result, vec![vec![0], vec![1]]);
    }

    #[test]
    fn assignment_many() {
        let (nr_robots, nr_requests) = (3, 11);
        let (plan, settings) = get_test_variables(nr_robots, nr_requests);
        let mut algorithm = MultiVehicleIlpFormulation::new(&plan, &settings);

        let requests = (0..nr_requests)
            .map(|i| {
                (
                    i as usize,
                    Request {
                        from: Vertex {
                            x: if i % 2 == 0 { 2 * i } else { 5 * i + 1 },
                            y: if i % 2 == 1 {
                                4 * i
                            } else {
                                nr_requests * 10 - 5 * i
                            },
                        },
                        to: Vertex {
                            x: if i % 2 != 0 {
                                nr_requests * 10 - 5 * i
                            } else {
                                2 * i + 4
                            },
                            y: if i % 2 == 0 {
                                3 * i
                            } else {
                                nr_requests * 8 - 2 * i
                            },
                        },
                    },
                )
            })
            .collect();
        let availability = (0..nr_robots)
            .map(|i| {
                (
                    i,
                    Vertex {
                        x: (i as u64 * 31) % 7 + 12,
                        y: (i as u64 * 23) % 11 + 3,
                    },
                )
            })
            .collect();

        let result = algorithm.calculate_assignment(&requests, &availability);
        assert_eq!(
            result.into_iter().map(|v| v.len() as u64).sum::<u64>(),
            nr_requests
        );
    }
}
