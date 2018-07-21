use algorithm::assignment::AssignmentAlgorithm;
use algorithm::path::PathAlgorithm;
use algorithm::NoSolutionError;
use algorithm::DAT_FILE_NAME;
use algorithm::RUN_FILE_NAME;
use fnv::FnvHashMap;
use fnv::FnvHashSet;
use simulation::plan::Plan;
use simulation::plan::UndirectedEdge;
use simulation::plan::Vertex;
use simulation::settings::Settings;
use simulation::state::History;
use simulation::state::State;
use simulation::Instructions;
use simulation::MoveInstruction;
use simulation::PlacementInstruction;
use simulation::RemovalInstruction;
use std::env::temp_dir;
use std::fs::create_dir;
use std::fs::remove_dir_all;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::path::PathBuf;
use std::process::Command;

const MOD_FILE_PATH: &str = "/home/bram/git/disjoint-path-routing/src/algorithm/path/ilp/ilp.mod";
const WORKING_DIRECTORY: &str = "path_ilp";

pub struct ILPSteps<'p, 's, 'a> {
    settings: &'s Settings,
    plan: &'p Plan,
    assignment_algorithm: Box<AssignmentAlgorithm<'p, 's> + 'a>,

    steps_at_once: u64,

    pub assignment: Vec<Vec<usize>>,
    assignment_initialized: bool,
}

impl<'p, 's, 'a> ILPSteps<'p, 's, 'a> {
    pub fn new(
        plan: &'p impl Plan,
        settings: &'s Settings,
        assignment_algorithm: Box<impl AssignmentAlgorithm<'p, 's> + 'a>,
        steps_at_once: u64,
    ) -> ILPSteps<'p, 's, 'a> {
        ILPSteps {
            settings,
            plan,
            assignment_algorithm,

            steps_at_once,
            assignment: Vec::new(),
            assignment_initialized: false,
        }
    }
    fn get_paths() -> (PathBuf, PathBuf, PathBuf, PathBuf) {
        let model_path = <&str as AsRef<Path>>::as_ref(&MOD_FILE_PATH).to_path_buf();

        let working_directory = temp_dir().join(WORKING_DIRECTORY);
        let dat_path = working_directory.join(DAT_FILE_NAME);
        let run_path = working_directory.join(RUN_FILE_NAME);

        (working_directory, dat_path, run_path, model_path)
    }
    /// Robot, Time, List of locations
    /// Robot, list of (location, cost)
    fn calculate_parameters(
        &self,
        state: &State,
    ) -> (
        Vec<Vec<Vec<Vertex>>>,
        FnvHashSet<UndirectedEdge>,
        Vec<Vec<(Vertex, u64)>>,
    ) {
        let locations = (0..self.settings.nr_robots)
            .map(|robot| {
                (0..(self.steps_at_once) + 1)
                    .map(|time| {
                        self.plan
                            .neighborhood(state.robot_states[robot].vertex.unwrap(), time)
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();

        let edges = self.plan.edges();

        let mut costs = Vec::new();
        for (robot, times) in locations.iter().enumerate() {
            let mut robot_locations_costs = Vec::new();
            if let Some(request_id) = self.assignment[robot].first() {
                let request = state.requests.get(&request_id).unwrap();
                let goal_vertex = if state.robot_states[robot].parcel_id.is_some() {
                    request.to
                } else {
                    request.from
                };
                for &location in times.last().unwrap() {
                    robot_locations_costs
                        .push((location, self.plan.path_length(location, goal_vertex)));
                }
            } else {
                for &location in times.last().unwrap() {
                    robot_locations_costs.push((location, 0));
                }
            }
            costs.push(robot_locations_costs);
        }

        (locations, edges, costs)
    }
    fn write_data_file(
        &self,
        path: impl AsRef<Path>,
        locations: Vec<Vec<Vec<Vertex>>>,
        edges: FnvHashSet<UndirectedEdge>,
        robot_costs: Vec<Vec<(Vertex, u64)>>,
    ) -> FnvHashMap<usize, Vertex> {
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

        writeln!(file, "set EDGES :=");
        for UndirectedEdge { first, second } in edges {
            if vertex_to_usize.contains_key(&first) && vertex_to_usize.contains_key(&second) {
                let v1 = vertex_to_usize.get(&first).unwrap();
                let v2 = vertex_to_usize.get(&second).unwrap();
                writeln!(file, "  {} {},", v1, v2);
                writeln!(file, "  {} {},", v2, v1);
            }
        }
        for &id in usize_to_vertex.keys() {
            writeln!(file, "  {} {},", id, id);
        }
        writeln!(file, ";");

        for (robot, times) in locations.iter().enumerate() {
            for (time, locations) in times.iter().enumerate() {
                writeln!(file, "set TIMES_ROBOTS_LOCATIONS[{}, {}] :=", time, robot);
                for location in locations {
                    writeln!(file, "  {},", vertex_to_usize.get(location).unwrap());
                }
                writeln!(file, ";");
            }
        }

        writeln!(file, "param cost :=");
        for (robot, location_costs) in robot_costs.iter().enumerate() {
            for &(location, cost) in location_costs {
                writeln!(
                    file,
                    "{}\t{}\t{}",
                    robot,
                    vertex_to_usize.get(&location).unwrap(),
                    cost
                );
            }
        }
        writeln!(file, ";");

        usize_to_vertex
    }
    fn get_vertex_enumeration(
        locations: &Vec<Vec<Vec<Vertex>>>,
    ) -> (FnvHashMap<Vertex, usize>, FnvHashMap<usize, Vertex>) {
        let mut all_locations = FnvHashSet::default();
        for robots in locations.iter() {
            for &location in robots.last().unwrap().iter() {
                all_locations.insert(location);
            }
        }

        let (mut vertex_to_usize, mut usize_to_vertex) =
            (FnvHashMap::default(), FnvHashMap::default());
        for (id, &location) in all_locations.iter().enumerate() {
            usize_to_vertex.insert(id, location);
            vertex_to_usize.insert(location, id);
        }

        (vertex_to_usize, usize_to_vertex)
    }
    fn write_run_file<T>(path: T, model_path: T, data_path: T)
    where
        T: AsRef<Path>,
    {
        let mut file = File::create(path).unwrap();

        writeln!(file, "model '{}';", model_path.as_ref().to_str().unwrap());
        writeln!(file, "data '{}';", data_path.as_ref().to_str().unwrap());
        writeln!(
            file,
            "option solver '/home/bram/Downloads/amplide.linux64/gurobi';"
        );
        writeln!(file, "option show_stats 0;");
        writeln!(file, "option gurobi_options 'timelim 1';");
        writeln!(file, "solve;");
        writeln!(file, "option omit_zero_rows 1;");
        file.write("display {r in ROBOTS, i in TIMES_ROBOTS_LOCATIONS[1, r]} Time_Robot_Location[1, r, i];\n".as_bytes());
    }
    fn execute_model(run_file_path: impl AsRef<Path>) -> String {
        String::from_utf8(
            Command::new("/home/bram/Downloads/amplide.linux64/ampl")
                .arg(run_file_path.as_ref())
                .output()
                .unwrap()
                .stdout,
        ).unwrap()
    }
    fn parse_ampl_output(output: String) -> Vec<usize> {
        let first_lines = output.lines().collect::<Vec<_>>();
        let first_split = first_lines.split(|&line| line == ";").next().unwrap();
        let lines = first_split
            .iter()
            .skip_while(|line| !line.starts_with("Time_Robot_Location["))
            .skip(1)
            .collect::<Vec<_>>();

        let mut end_locations = lines
            .into_iter()
            .map(
                |line| match line.split_whitespace().collect::<Vec<_>>().as_slice() {
                    [robot, location, "1"] => {
                        let robot = robot.parse::<usize>().unwrap();
                        let location = location.parse::<usize>().unwrap();

                        (robot, location)
                    }
                    _ => panic!(),
                },
            )
            .collect::<Vec<_>>();

        end_locations.sort_by_key(|&(robot, _)| robot);
        end_locations
            .into_iter()
            .map(|(_, location)| location)
            .collect()
    }
    fn get_instructions(&mut self, history: &History, new_locations: Vec<Vertex>) -> Instructions {
        let mut instructions = Instructions {
            movements: Vec::new(),
            placements: Vec::new(),
            removals: Vec::new(),
            robot_removals: Vec::new(),
        };

        for (robot_id, new_location) in new_locations.into_iter().enumerate() {
            let previous_state = history.last_robot_state(robot_id);
            let previous_location = previous_state.vertex.unwrap();
            if let Some(parcel) = previous_state.parcel_id {
                let goal_location = history.last_state().requests.get(&parcel).unwrap().to;
                if previous_location == goal_location {
                    instructions.removals.push(RemovalInstruction {
                        parcel,
                        robot_id,
                        vertex: previous_location,
                    });
                    self.assignment[robot_id].remove(0);
                } else {
                    instructions.movements.push(MoveInstruction {
                        robot_id,
                        vertex: new_location,
                    });
                }
            } else {
                if let Some(request_id) = self.assignment[robot_id].first() {
                    let goal_location =
                        history.last_state().requests.get(&request_id).unwrap().from;
                    if previous_location == goal_location {
                        instructions.placements.push(PlacementInstruction {
                            robot_id,
                            parcel: *request_id,
                            vertex: goal_location,
                        });
                    } else {
                        instructions.movements.push(MoveInstruction {
                            robot_id,
                            vertex: new_location,
                        });
                    }
                } else {
                    if new_location != previous_location {
                        instructions.movements.push(MoveInstruction {
                            robot_id,
                            vertex: new_location,
                        });
                    }
                }
            }
        }

        instructions
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
                .map(|state| (0, state.vertex.unwrap()))
                .collect::<Vec<_>>();
            self.assignment = self
                .assignment_algorithm
                .calculate_assignment(requests, &availability);
            self.assignment_initialized = true;
            //            println!("{:?}", self.assignment);
        }

        let (locations, edges, costs) = self.calculate_parameters(history.last_state());
        let (working_directory, data_path, run_file_path, model_path) = ILPSteps::get_paths();
        if working_directory.exists() {
            remove_dir_all(working_directory.as_path());
        }
        create_dir(working_directory);
        let vertex_map = self.write_data_file(data_path.as_path(), locations, edges, costs);
        //        for (id, vertex) in vertex_map.iter() {
        //            println!("{}, {:?}", id, vertex);
        //        }
        ILPSteps::write_run_file(
            run_file_path.as_path(),
            model_path.as_path(),
            data_path.as_path(),
        );
        let output = ILPSteps::execute_model(run_file_path.as_path());
        let new_positions = ILPSteps::parse_ampl_output(output);
        let new_positions = new_positions
            .into_iter()
            .map(|id| *vertex_map.get(&id).unwrap())
            .collect();
        let instructions = self.get_instructions(history, new_positions);

        instructions
    }
}

#[cfg(test)]
mod test {

    use fnv::FnvHashMap;

    use algorithm::assignment::greedy_makespan::GreedyMakespan;
    use algorithm::assignment::multiple_vehicle_ilp::MultiVehicleIlpFormulation;
    use algorithm::path::ilp::ILPSteps;
    use algorithm::path::PathAlgorithm;
    use simulation::demand::Request;
    use simulation::plan::one_three_rectangle::OneThreeRectangle;
    use simulation::plan::Vertex;
    use simulation::settings::Settings;
    use simulation::state::History;
    use simulation::state::RobotState;
    use simulation::state::State;
    use simulation::MoveInstruction;

    #[test]
    fn test_have_parcel() {
        let plan = OneThreeRectangle::new(10, 10);
        let settings = Settings {
            total_time: 10,
            nr_robots: 1,
            nr_requests: 1,
            output_file: None,
        };
        let requests = map!
        [
            0 => Request {
                from: Vertex { x: 0, y: 0 },
                to: Vertex { x: 2, y: 2 },
            },
        ];
        let assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
        let mut algorithm = ILPSteps::new(&plan, &settings, assignment_algorithm, 3);

        let history = History {
            states: vec![State {
                robot_states: vec![RobotState {
                    robot_id: 0,
                    parcel_id: Some(0),
                    vertex: Some(Vertex { x: 2, y: 0 }),
                }],
                requests,
            }],
            calculation_times: Vec::new(),
        };

        assert_eq!(
            algorithm.next_step(&history).movements,
            vec![MoveInstruction {
                robot_id: 0,
                vertex: Vertex { x: 2, y: 1 },
            }]
        );
    }

    #[test]
    fn test_no_parcel() {
        let plan = OneThreeRectangle::new(10, 10);
        let settings = Settings {
            total_time: 10,
            nr_robots: 1,
            nr_requests: 1,
            output_file: None,
        };
        let requests = map!
        [
            0 => Request {
                from: Vertex { x: 0, y: 0 },
                to: Vertex { x: 2, y: 2 },
            },
        ];
        let assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
        let mut algorithm = ILPSteps::new(&plan, &settings, assignment_algorithm, 3);

        let history = History {
            states: vec![State {
                robot_states: vec![RobotState {
                    robot_id: 0,
                    parcel_id: None,
                    vertex: Some(Vertex { x: 2, y: 0 }),
                }],
                requests,
            }],
            calculation_times: Vec::new(),
        };

        assert_eq!(
            algorithm.next_step(&history).movements,
            vec![MoveInstruction {
                robot_id: 0,
                vertex: Vertex { x: 1, y: 0 },
            }]
        );
    }

    #[test]
    fn test_two_robots() {
        let plan = OneThreeRectangle::new(10, 10);
        let settings = Settings {
            total_time: 10,
            nr_robots: 2,
            nr_requests: 2,
            output_file: None,
        };
        let requests = map!
        [
            0 => Request {
                from: Vertex { x: 0, y: 0, },
                to: Vertex { x: 2, y: 2, },
            },
            1 => Request {
                from: Vertex { x: 4, y: 4, },
                to: Vertex { x: 6, y: 6, },
            },
        ];
        let assignment_algorithm = Box::new(MultiVehicleIlpFormulation::new(&plan, &settings));
        let mut algorithm = ILPSteps::new(&plan, &settings, assignment_algorithm, 1);

        let history = History {
            states: vec![State {
                robot_states: vec![
                    RobotState {
                        robot_id: 0,
                        parcel_id: None,
                        vertex: Some(Vertex { x: 1, y: 0 }),
                    },
                    RobotState {
                        robot_id: 1,
                        parcel_id: None,
                        vertex: Some(Vertex { x: 4, y: 2 }),
                    },
                ],
                requests,
            }],
            calculation_times: Vec::new(),
        };

        assert_eq!(
            algorithm.next_step(&history).movements,
            vec![
                MoveInstruction {
                    robot_id: 0,
                    vertex: Vertex { x: 0, y: 0 },
                },
                MoveInstruction {
                    robot_id: 1,
                    vertex: Vertex { x: 4, y: 3 },
                },
            ]
        );
    }
}
