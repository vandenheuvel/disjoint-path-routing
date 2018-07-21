use itertools::Itertools;

use algorithm::assignment::greedy_makespan::GreedyMakespan;
use algorithm::assignment::AssignmentAlgorithm;
use algorithm::assignment::LPIOError;
use algorithm::DAT_FILE_NAME;
use algorithm::RUN_FILE_NAME;
use fnv::FnvHashMap;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::plan::Vertex;
use simulation::settings::Settings;
use std::convert::AsRef;
use std::env::temp_dir;
use std::fs::create_dir;
use std::fs::remove_dir_all;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::path::PathBuf;
use std::process::Command;
use std::thread::spawn;
use std::thread::JoinHandle;

const MOD_FILE_PATH: &str = "/home/bram/git/disjoint-path-routing/src/algorithm/assignment/makespan_single_vehicle_ilp/makespan_single_vehicle_ilp.mod";
const WORKING_DIRECTORY: &str = "makespan_single_vehicle_ilp";

pub struct MakespanSingleVehicleILP<'p, 's> {
    plan: &'p (dyn Plan + 'p),
    settings: &'s Settings,
}

impl<'p, 's> MakespanSingleVehicleILP<'p, 's> {
    pub fn new(plan: &'p impl Plan, settings: &'s Settings) -> MakespanSingleVehicleILP<'p, 's> {
        MakespanSingleVehicleILP { plan, settings }
    }
    fn calculate_optimal_request_order(
        &self,
        robot: usize,
        start_vertex: Vertex,
        assigned_requests: Vec<usize>,
        requests: &FnvHashMap<usize, Request>,
    ) -> JoinHandle<Vec<usize>> {
        let (model_path, working_directory, data_path, run_path) =
            MakespanSingleVehicleILP::get_paths(robot);
        MakespanSingleVehicleILP::create_working_directory(working_directory);
        let (first_distances, transition_distances, last_distances) =
            self.calculate_distances(start_vertex, &assigned_requests, requests);
        MakespanSingleVehicleILP::write_data_file(
            data_path.as_path(),
            &assigned_requests,
            first_distances,
            transition_distances,
            last_distances,
        );
        MakespanSingleVehicleILP::write_run_file(
            run_path.as_path(),
            model_path.as_path(),
            data_path.as_path(),
        );
        spawn(|| {
            let output = Command::new("/home/bram/Downloads/amplide.linux64/ampl")
                .arg(run_path)
                .output()
                .unwrap();
            let (first, transitions, last, _) =
                MakespanSingleVehicleILP::parse_ampl_output(output.stdout);
            MakespanSingleVehicleILP::reconstruct_request_order(first, transitions, last)
        })
    }
    fn create_working_directory(directory: PathBuf) {
        if directory.exists() {
            remove_dir_all(directory.as_path());
        }
        create_dir(directory).unwrap();
    }
    fn reconstruct_request_order(
        first: usize,
        ones: FnvHashMap<usize, usize>,
        last: usize,
    ) -> Vec<usize> {
        let mut requests = Vec::new();
        requests.push(first);

        if ones.contains_key(&first) {
            let mut current = first;

            while current != last {
                if let Some(request) = ones.get(&current) {
                    requests.push(*request);
                    current = *request;
                    if *request == last {
                        break;
                    }
                } else {
                    panic!()
                }
            }
        }

        requests
    }
    fn parse_ampl_output(output: Vec<u8>) -> (usize, FnvHashMap<usize, usize>, usize, u64) {
        let output = String::from_utf8(output).unwrap();
        let segments = output.lines().collect::<Vec<_>>();

        let objective = segments
            .iter()
            .skip_while(|line| !line.contains("objective"))
            .next()
            .unwrap()
            .split_whitespace()
            .last()
            .unwrap()
            .parse::<u64>()
            .unwrap();

        let mut segments = segments.split(|&line| line == ";");
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
        let first = get_lines("First_Request");
        let transitions = get_lines("Transition");
        let last = get_lines("Last_Request");

        let first = first[0]
            .split_whitespace()
            .next()
            .unwrap()
            .parse::<usize>()
            .unwrap();
        let transitions = transitions
            .into_iter()
            .map(
                |line| match line.split_whitespace().collect_vec().as_slice() {
                    [i, j, "1"] => {
                        let from = i.parse::<usize>().unwrap();
                        let to = j.parse::<usize>().unwrap();
                        (from, to)
                    }
                    _ => panic!(),
                },
            )
            .collect();
        let last = last[0]
            .split_whitespace()
            .next()
            .unwrap()
            .parse::<usize>()
            .unwrap();

        (first, transitions, last, objective)
    }
    fn write_data_file(
        path: impl AsRef<Path>,
        assigned_requests: &Vec<usize>,
        first_distances: Vec<(usize, u64)>,
        transition_distances: Vec<(usize, usize, u64)>,
        last_distances: Vec<(usize, u64)>,
    ) -> Result<(), LPIOError> {
        let mut file = File::create(path).unwrap();

        file.write("set REQUESTS :=\n".as_ref());
        for request in assigned_requests {
            file.write(format!("  {},\n", request).as_ref());
        }
        file.write(";\n".as_ref());

        file.write("param start_cost default 0 :=\n".as_ref());
        for (first, distance) in first_distances {
            if distance != 0 {
                file.write(format!("{}\t{}\n", first, distance).as_ref());
            }
        }
        file.write(";\n".as_ref());

        file.write("param transition_cost default 0 :=\n".as_ref());
        for (from, to, distance) in transition_distances {
            if distance != 0 {
                file.write(format!("{}\t{}\t{}\n", from, to, distance).as_ref());
            }
        }
        file.write(";\n".as_ref());

        file.write("param end_cost default 0 :=\n".as_ref());
        for (last, distance) in last_distances {
            if distance != 0 {
                file.write(format!("{}\t{}\n", last, distance).as_ref());
            }
        }
        file.write(";\n".as_ref());

        Ok(())
    }
    fn write_run_file(
        path: impl AsRef<Path>,
        model_path: impl AsRef<Path>,
        data_path: impl AsRef<Path>,
    ) -> Result<(), LPIOError> {
        let mut file = File::create(path).unwrap();

        writeln!(file, "model '{}';", model_path.as_ref().to_str().unwrap());
        writeln!(file, "data '{}';", data_path.as_ref().to_str().unwrap());
        file.write("option solver '/home/bram/Downloads/amplide.linux64/gurobi';\n".as_ref());
        writeln!(file, "option gurobi_options 'timelim {} bestbound 1';", 30);
        file.write("solve;\n".as_ref());
        file.write("option omit_zero_rows 1;\n".as_ref());
        file.write("option display_1col 1000000;\n".as_ref());
        file.write("display First_Request;\n".as_ref());
        file.write("display Transition;\n".as_ref());
        file.write("display Last_Request;\n".as_ref());

        Ok(())
    }
    fn calculate_distances(
        &self,
        start_vertex: Vertex,
        assigned_requests: &Vec<usize>,
        requests: &FnvHashMap<usize, Request>,
    ) -> (
        Vec<(usize, u64)>,
        Vec<(usize, usize, u64)>,
        Vec<(usize, u64)>,
    ) {
        let requests = assigned_requests
            .into_iter()
            .map(|request_id| (*request_id, requests.get(&request_id).unwrap()))
            .collect::<Vec<_>>();

        let first_distances = requests
            .iter()
            .map(|&(id, request)| (id as usize, start_vertex.distance(request.from)))
            .collect();
        let transition_distances = requests
            .iter()
            .cartesian_product(requests.iter())
            .map(|(&(i, request_i), &(j, request_j))| (i, j, request_i.to.distance(request_j.from)))
            .collect();
        let last_distances = requests
            .iter()
            .map(|&(id, request)| (id, self.calculate_last_distance(start_vertex, request.to)))
            .collect();

        (first_distances, transition_distances, last_distances)
    }
    fn calculate_last_distance(&self, _start_vertex: Vertex, _last_vertex: Vertex) -> u64 {
        0
    }
    fn get_paths(robot: usize) -> (PathBuf, PathBuf, PathBuf, PathBuf) {
        let model_path = <&str as AsRef<Path>>::as_ref(&MOD_FILE_PATH).to_path_buf();

        let mut working_directory = String::from(WORKING_DIRECTORY);
        working_directory.push('_');
        working_directory.push_str(&robot.to_string());
        let working_directory = temp_dir().join(working_directory);
        let dat_path = working_directory.join(DAT_FILE_NAME);
        let run_path = working_directory.join(RUN_FILE_NAME);

        (model_path, working_directory, dat_path, run_path)
    }
    pub fn calculate_assignment_quality(
        &mut self,
        requests: &FnvHashMap<usize, Request>,
        availability: &Vec<(usize, Vertex)>,
    ) -> u64 {
        let mut makespan_assignment = GreedyMakespan::new(self.plan, self.settings);
        let assignment = makespan_assignment.calculate_assignment(requests, availability);

        let handles = assignment
            .into_iter()
            .enumerate()
            .map(|(index, assigned)| {
                let (robot, start_vertex) = availability[index];
                let (model_path, working_directory, data_path, run_path) =
                    MakespanSingleVehicleILP::get_paths(robot);
                MakespanSingleVehicleILP::create_working_directory(working_directory);
                let (first_distances, transition_distances, last_distances) =
                    self.calculate_distances(start_vertex, &assigned, requests);
                MakespanSingleVehicleILP::write_data_file(
                    data_path.as_path(),
                    &assigned,
                    first_distances,
                    transition_distances,
                    last_distances,
                );
                MakespanSingleVehicleILP::write_run_file(
                    run_path.as_path(),
                    model_path.as_path(),
                    data_path.as_path(),
                );
                (
                    spawn(move || {
                        let output = Command::new("/home/bram/Downloads/amplide.linux64/ampl")
                            .arg(run_path)
                            .output()
                            .unwrap();
                        let (_, _, _, obj) =
                            MakespanSingleVehicleILP::parse_ampl_output(output.stdout);
                        obj
                    }),
                    assigned
                        .iter()
                        .map(|r| requests.get(r).unwrap().distance())
                        .sum::<u64>(),
                )
            })
            .collect::<Vec<_>>();
        handles
            .into_iter()
            .map(|(handle, sum)| handle.join().ok().unwrap() + sum)
            .max()
            .unwrap()
    }
}

impl<'p, 's> AssignmentAlgorithm<'p, 's> for MakespanSingleVehicleILP<'p, 's> {
    fn calculate_assignment(
        &mut self,
        requests: &FnvHashMap<usize, Request>,
        availability: &Vec<(usize, Vertex)>,
    ) -> Vec<Vec<usize>> {
        let mut makespan_assignment = GreedyMakespan::new(self.plan, self.settings);
        let assignment = makespan_assignment.calculate_assignment(requests, availability);

        let handles = assignment
            .into_iter()
            .enumerate()
            .map(|(index, assigned)| {
                let (robot, start_vertex) = availability[index];
                self.calculate_optimal_request_order(robot, start_vertex, assigned, requests)
            })
            .collect::<Vec<_>>();
        handles
            .into_iter()
            .map(JoinHandle::join)
            .map(Result::ok)
            .map(Option::unwrap)
            .collect()
    }
}

#[cfg(test)]
mod test {

    use algorithm::assignment::makespan_single_vehicle_ilp::MakespanSingleVehicleILP;
    use algorithm::assignment::test::get_test_variables;
    use algorithm::assignment::AssignmentAlgorithm;
    use fnv::FnvHashMap;
    use simulation::demand::Request;
    use simulation::plan::Vertex;

    #[test]
    fn assignment_one() {
        let (plan, settings) = get_test_variables(1, 1);
        let mut algorithm = MakespanSingleVehicleILP::new(&plan, &settings);

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
        let (plan, settings) = get_test_variables(1, 1);
        let mut algorithm = MakespanSingleVehicleILP::new(&plan, &settings);

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
        let availability = vec![(1, Vertex { x: 0, y: 0 })];

        let result = algorithm.calculate_assignment(&requests, &availability);
        assert_eq!(result, vec![vec![0, 1]]);
    }

    #[test]
    fn assignment_many() {
        let nr_requests = 200;
        let (plan, settings) = get_test_variables(1, nr_requests);
        let mut algorithm = MakespanSingleVehicleILP::new(&plan, &settings);

        let requests = (0..nr_requests)
            .map(|i| {
                (
                    i as usize,
                    Request {
                        from: Vertex { x: 0, y: i },
                        to: Vertex { x: 0, y: i + 1 },
                    },
                )
            })
            .collect();
        let availability = vec![(2, Vertex { x: 0, y: 0 })];

        let result = algorithm.calculate_assignment(&requests, &availability);
        assert_eq!(
            result,
            vec![(0..nr_requests).map(|i| i as usize).collect::<Vec<_>>()]
        );
    }

    #[test]
    fn assignment_all_equal() {
        let nr_requests = 50;
        let (plan, settings) = get_test_variables(1, nr_requests);
        let mut algorithm = MakespanSingleVehicleILP::new(&plan, &settings);

        let requests = (0..nr_requests)
            .map(|i| {
                (
                    i as usize,
                    Request {
                        from: Vertex { x: 0, y: 0 },
                        to: Vertex { x: 0, y: 2 },
                    },
                )
            })
            .collect();
        let availability = vec![(3, Vertex { x: 0, y: 0 })];

        let result = algorithm.calculate_assignment(&requests, &availability);
    }

    #[test]
    fn separate_cluster() {
        let (plan, settings) = get_test_variables(1, 4);
        let mut algorithm = MakespanSingleVehicleILP::new(&plan, &settings);

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
            2 => Request {
                from: Vertex { x: 10, y: 2, },
                to: Vertex { x: 10, y: 3, },
            },
            3 => Request {
                from: Vertex { x: 10, y: 3, },
                to: Vertex { x: 10, y: 2, },
            },
        ];
        let availability = vec![(4, Vertex { x: 0, y: 0 })];

        let result = algorithm.calculate_assignment(&requests, &availability);
        assert_eq!(result, vec![vec![0, 1, 2, 3]]);
    }
}
