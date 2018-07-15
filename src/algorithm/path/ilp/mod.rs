use algorithm::path::PathAlgorithm;
use simulation::settings::Settings;
use algorithm::assignment::AssignmentAlgorithm;
use algorithm::NoSolutionError;
use simulation::state::History;
use simulation::Instructions;
use simulation::plan::Plan;
use simulation::state::State;
use std::path::PathBuf;
use std::path::Path;
use std::env::temp_dir;
use algorithm::DAT_FILE_NAME;
use algorithm::RUN_FILE_NAME;
use std::fs::create_dir;
use simulation::plan::Vertex;
use fnv::FnvHashMap;
use std::fs::File;
use fnv::FnvHashSet;
use std::io::Write;
use std::process::Command;


const MOD_FILE_PATH: &str = "/home/bram/git/disjoint-path-routing/src/algorithm/path/ilp/ilp.mod";
const WORKING_DIRECTORY: &str = "path_ilp";


pub struct ILPSteps<'p, 's, 'a> {
    settings: &'s Settings,
    plan: &'p Plan,
    assignment_algorithm: Box<AssignmentAlgorithm<'p, 's> + 'a>,

    steps_at_once: u64,

    assignment: Vec<Vec<usize>>,
    assignment_initialized: bool,
}

impl <'p, 's, 'a> ILPSteps<'p, 's, 'a> {
    fn new(
        plan: &'p impl Plan,
        assignment_algorithm: Box<impl AssignmentAlgorithm<'p, 's> + 'a>,
        settings: &'s Settings,
        steps_at_once: u64,
    ) -> ILPSteps<'p, 's, 'a> {
        ILPSteps {
            settings,
            plan,
            assignment_algorithm,

            steps_at_once: 3,
            assignment: Vec::new(),
            assignment_initialized: false,
        }
    }
    fn get_paths() -> (PathBuf, PathBuf, PathBuf, PathBuf) {
        let model_path = <&str as AsRef<Path>>::as_ref(&MOD_FILE_PATH).to_path_buf();

        let working_directory = temp_dir().join(WORKING_DIRECTORY);
        let dat_path = working_directory.join(DAT_FILE_NAME);
        let run_path = working_directory.join(RUN_FILE_NAME);

        (model_path, working_directory, dat_path, run_path)
    }
    /// Robot, Time, List of locations
    /// Robot, list of (location, cost)
    fn calculate_parameters(&self, state: &State) -> (Vec<Vec<Vec<Vertex>>>, Vec<Vec<(Vertex, u64)>>) {
        let locations = (0..self.settings.nr_robots)
            .map(|robot| {
                (0..self.steps_at_once)
                    .map(|time| self.plan.neighborhood(state.robot_states[robot].vertex, time))
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();

        let mut costs = Vec::new();
        for (robot, times) in locations.iter().enumerate() {
            let mut robot_locations_costs = Vec::new();
            for &location in times.last().unwrap() {
                let goal_vertex = state.requests.get(&self.assignment[robot][0]).unwrap().from;
                robot_locations_costs.push((location, self.plan.path_length(location, goal_vertex)));
            }
            costs.push(robot_locations_costs);
        }

        (locations, costs)
    }
    fn write_data_file(&self, path: impl AsRef<Path>, locations: Vec<Vec<Vec<Vertex>>>, robot_costs: Vec<Vec<(Vertex, u64)>>) -> FnvHashMap<usize, Vertex> {
        let mut file = File::create(path).unwrap();

        writeln!(file, "param T := {};", self.steps_at_once);

        writeln!(file, "set ROBOTS :=");
        for r in 0..self.settings.nr_robots {
            writeln!(file, "  {},", r);
        }
        writeln!(file, ";");

        let (vertex_to_usize, usize_to_vertex) = ILPSteps::get_vertex_enumeration(&locations);
        writeln!(file, "set LOCATIONS :=");
        for location_id in usize_to_vertex.keys() {
            writeln!(file, "  {},", location_id);
        }
        writeln!(file, ";");

        for (robot, times) in locations.iter().enumerate() {
            for (time, locations) in times.iter().enumerate() {
                writeln!(file, "set TIMES_ROBOT_LOCATIONS[{}, {}] :=", time, robot);
                for location in locations {
                    writeln!(file, "  {},", vertex_to_usize.get(location).unwrap());
                }
                writeln!(file, ";");
            }
        }

        writeln!(file, "param cost :=");
        for (robot, location_costs) in robot_costs.iter().enumerate() {
            for &(location, cost) in location_costs {
                writeln!(file, "{}\t{}\t{}", robot, vertex_to_usize.get(&location).unwrap(), cost);
            }
        }
        writeln!(file, ";");

        usize_to_vertex
    }
    fn get_vertex_enumeration(locations: &Vec<Vec<Vec<Vertex>>>) -> (FnvHashMap<Vertex, usize>, FnvHashMap<usize, Vertex>) {
        let mut all_locations = FnvHashSet::default();
        for robots in locations.iter() {
            for &location in robots.last().unwrap().iter() {
                all_locations.insert(location);
            }
        }

        let (mut vertex_to_usize, mut usize_to_vertex) = (FnvHashMap::default(), FnvHashMap::default());
        for (id, &location) in all_locations.iter().enumerate() {
            usize_to_vertex.insert(id, location);
            vertex_to_usize.insert(location, id);
        }

        (vertex_to_usize, usize_to_vertex)
    }
    fn write_run_file<T>(path: T, model_path: T, data_path: T) where T: AsRef<Path> {
        let mut file = File::create(path).unwrap();

        writeln!(file, "model '{}';", model_path.as_ref().to_str().unwrap());
        writeln!(file, "data '{}';", data_path.as_ref().to_str().unwrap());
        writeln!(file, "option solver '/home/bram/Downloads/amplide.linux64/gurobi';");
        writeln!(file, "option show_stats 0;");
        writeln!(file, "gurobi_options 'timelim 1';");
        writeln!(file, "solve;");
        writeln!(file, "option omit_zero_rows 1;");
        writeln!(file, "display Time_Robot_Location'");
        file.write("display {r in ROBOTS, i in TIMES_ROBOTS_LOCATIONS[1, r]} Time_Robot_Location[T, r, i];\n".as_bytes());
    }
    fn execute_model(run_file_path: impl AsRef<Path>) -> String {
        String::from_utf8(Command::new("/home/bram/Downloads/amplide.linux64/ampl")
            .arg(run_file_path.as_ref())
            .output()
            .unwrap().stdout).unwrap()
    }
    fn parse_ampl_output(output: String) -> Vec<usize> {
        let lines = output
            .lines()
            .collect::<Vec<_>>()
            .split(|line| line == ";")
            .next()
            .unwrap()
            .iter()
            .skip_while(|line| !line.starts_with("Time_Robot_Location["))
            .collect::<Vec<_>>();

        let mut end_locations = lines
            .into_iter()
            .map(|line| match line.split_whitespace().collect::<Vec<_>>().as_slice() {
                [robot, location, "1"] => {
                    let robot = robot.parse::<usize>().unwrap();
                    let location = locdation.parse::<usize>().unwrap();

                    (robot, location)
                },
                _ => panic!(),
            })
            .collect::<Vec<_>>();

        end_locations.sort_by_key(|(robot, _)| robot);

        end_locations.into_iter().map(|(_, location)| location).collect()
    }
    fn get_instructions(history: &History, new_locations: Vec<Vertex>) -> Instructions {
        unimplemented!();
    }
}

impl<'p, 's, 'a> PathAlgorithm<'p, 's, 'a> for ILPSteps<'p, 's, 'a> {
    fn initialize(&mut self) -> Result<(), NoSolutionError> {
        Ok(())
    }

    fn next_step(&mut self, history: &History) -> Instructions {
        if !self.assignment_initialized {
            let requests = &history.last_state().requests;
            let availability = history
                .last_state()
                .robot_states
                .iter()
                .map(|state| (0, state.vertex))
                .collect::<Vec<_>>();
            self.assignment = self.assignment_algorithm.calculate_assignment(requests, &availability);
            self.assignment_initialized = true;
        }

        let (locations, costs) = self.calculate_parameters(history.last_state());
        let (working_directory, data_path, run_file_path, model_path) = ILPSteps::get_paths();
        if !working_directory.exists() {
            create_dir(working_directory);
        }
        let vertex_map = self.write_data_file(data_path.as_path(), locations, costs);
        ILPSteps::write_run_file(run_file_path.as_path(), model_path.as_path(), data_path.as_path());
        let output = ILPSteps::execute_model(run_file_path.as_path());
        let new_positions = ILPSteps::parse_ampl_output(output);
        let new_positions = new_positions
            .into_iter()
            .map(|id| vertex_map.get(&id).unwrap())
            .collect();
        let instructions = ILPSteps::get_instructions(history, new_states);

        instructions
    }
}
