use algorithm::path::PathAlgorithm;
use fnv::FnvHashMap;
use fnv::FnvHashSet;
use rand::seq::sample_iter;
use rand::thread_rng;
use simulation::demand::Demand;
use simulation::demand::Request;
use simulation::plan::Plan;
use simulation::plan::Vertex;
use simulation::settings::Settings;
use simulation::state::History;
use simulation::state::RobotState;
use simulation::state::State;
use simulation::IllegalInstructionError;
use simulation::IllegalMoveError;
use simulation::IllegalPlacementError;
use simulation::IllegalRemovalError;
use simulation::Instructions;
use simulation::MoveInstruction;
use simulation::ParcelInstruction;
use simulation::PlacementInstruction;
use simulation::RemovalInstruction;
use simulation::RobotRemovalInstruction;
use std::fs::File;
use std::fs::OpenOptions;
use std::io;
use std::io::BufWriter;

pub struct Simulation<'a, 'p, 's> {
    algorithm: Box<PathAlgorithm<'p, 's, 'a> + 'a>,
    plan: &'p Plan,
    demand: Box<Demand + 'a>,
    settings: &'s Settings,

    pub history: History,
    output_writer: Option<BufWriter<File>>,
}

impl<'a, 'p, 's> Simulation<'a, 'p, 's> {
    pub fn new(
        algorithm: Box<PathAlgorithm<'p, 's, 'a> + 'a>,
        plan: &'p Plan,
        demand: Box<Demand + 'a>,
        settings: &'s Settings,
    ) -> Simulation<'a, 'p, 's> {
        Simulation {
            algorithm,
            plan,
            demand,
            settings,

            history: History::empty(),
            output_writer: None,
        }
    }
    /// Gets the initial state of the system set up, creates history of time 0.
    pub fn initialize(&mut self) -> io::Result<()> {
        if let Some(ref file_name) = self.settings.output_file {
            self.setup_output(file_name)?;
        }
        self.set_initial_state();
        self.algorithm.initialize();

        if let Some(ref mut writer) = self.output_writer {
            self.settings.write(writer)?;
            self.plan.write(writer)?;
        }

        Ok(())
    }
    fn setup_output(&mut self, output_file_name: &String) -> io::Result<()> {
        let file = OpenOptions::new().write(true).open(output_file_name)?;
        let buffered_writer = BufWriter::new(file);

        Ok(self.output_writer = Some(buffered_writer))
    }
    fn set_initial_state(&mut self) {
        let mut rng = thread_rng();
        let sample = sample_iter(
            &mut rng,
            self.plan.vertices().into_iter(),
            self.settings.nr_robots,
        ).unwrap();
        let robot_states = sample
            .into_iter()
            .enumerate()
            .map(|(robot_id, vertex)| RobotState {
                robot_id,
                vertex: Some(vertex),
                parcel_id: None,
            })
            .collect();

        let requests = self
            .demand
            .generate(self.plan, self.settings.nr_requests)
            .into_iter()
            .enumerate()
            .collect();

        self.history.states.push(State {
            robot_states,
            requests,
        });
    }
    pub fn run(mut self) -> Result<History, Box<IllegalInstructionError>> {
        while self.history.last_state().requests.len() > 0
            && self.history.time() < self.settings.total_time
        {
            println!("{:?}", self.history.last_state());
            println!("{}", self.history.time());
            let instructions = self.algorithm.next_step(&self.history);
            self.new_state(instructions)?;
            if let Some(ref mut writer) = self.output_writer {
                self.history.last_state().write(writer);
            }
        }

        Ok(self.history)
    }
    fn new_state(
        &mut self,
        instructions: Instructions,
    ) -> Result<(), Box<IllegalInstructionError>> {
        let used_vertices = self
            .history
            .last_state()
            .robot_states
            .iter()
            .map(|robot| (robot.vertex.unwrap(), robot.robot_id))
            .collect::<FnvHashMap<_, _>>();
        let mut newly_used_vertices = FnvHashSet::default();

        let mut new_states = self.history.last_state().robot_states.clone();
        let mut new_requests = self.history.last_state().requests.clone();

        self.process_move_instructions(
            instructions.movements,
            &mut new_states,
            &used_vertices,
            &mut newly_used_vertices,
        ).map_err(|e| Box::new(e) as Box<IllegalInstructionError>)?;
        self.process_placement_instructions(
            instructions.placements,
            &mut new_states,
            &used_vertices,
            &mut newly_used_vertices,
            &mut new_requests,
        ).map_err(|e| Box::new(e) as Box<IllegalInstructionError>)?;
        self.process_removal_instructions(
            instructions.removals,
            &mut new_states,
            &mut new_requests,
            &mut newly_used_vertices,
        ).map_err(|e| Box::new(e) as Box<IllegalInstructionError>)?;
        self.process_robot_removal_instructions(instructions.robot_removals, &mut new_states);

        Ok(self.history.states.push(State {
            robot_states: new_states,
            requests: new_requests,
        }))
    }
    fn process_move_instructions(
        &self,
        move_instructions: Vec<MoveInstruction>,
        new_states: &mut Vec<RobotState>,
        used_vertices: &FnvHashMap<Vertex, usize>,
        newly_used_vertices: &mut FnvHashSet<Vertex>,
    ) -> Result<(), IllegalMoveError> {
        for instruction in move_instructions {
            if self
                .history
                .last_robot_state(instruction.robot_id)
                .vertex
                .is_none()
            {
                continue;
            }

            if let Some(error) = self.check_for_move_instruction_error(
                instruction,
                used_vertices,
                newly_used_vertices,
            ) {
                return Err(error);
            }
            let MoveInstruction { robot_id, vertex } = instruction;

            new_states[robot_id] = RobotState {
                robot_id,
                parcel_id: self.history.last_robot_state(robot_id).parcel_id,
                vertex: Some(vertex),
            };
            newly_used_vertices.insert(vertex);
        }

        Ok(())
    }
    fn check_for_move_instruction_error(
        &self,
        instruction: MoveInstruction,
        used_vertices: &FnvHashMap<Vertex, usize>,
        newly_used_vertices: &FnvHashSet<Vertex>,
    ) -> Option<IllegalMoveError> {
        let MoveInstruction { robot_id, vertex } = instruction;

        //        match used_vertices.get(&vertex) {
        //            None => (),
        //            Some(robot) if *robot != instruction.robot_id => {
        //                return Some(IllegalMoveError::from(
        //                    instruction,
        //                    "State used in previous time step by other robot".to_string(),
        //                    self.history.time(),
        //                ));
        //            }
        //            _ => (),
        //        }
        //        if newly_used_vertices.contains(&vertex) {
        //            return Some(IllegalMoveError::from(
        //                instruction,
        //                "State will be used in next time step".to_string(),
        //                self.history.time(),
        //            ));
        //        }
        if self
            .history
            .last_robot_state(robot_id)
            .vertex
            .unwrap()
            .distance(vertex) > 1
        {
            return Some(IllegalMoveError {
                instruction,
                message: "Only one move at a time".to_string(),
                time: self.history.time(),
            });
        }

        None
    }
    fn process_placement_instructions(
        &self,
        placement_instructions: Vec<PlacementInstruction>,
        new_states: &mut Vec<RobotState>,
        used_vertices: &FnvHashMap<Vertex, usize>,
        newly_used_vertices: &mut FnvHashSet<Vertex>,
        new_requests: &mut FnvHashMap<usize, Request>,
    ) -> Result<(), IllegalPlacementError> {
        for instruction in placement_instructions {
            if self
                .history
                .last_robot_state(instruction.robot_id)
                .vertex
                .is_none()
            {
                continue;
            }
            if let Some(error) = self.check_for_placement_instruction_error(
                instruction,
                used_vertices,
                newly_used_vertices,
                new_requests,
            ) {
                return Err(error);
            }

            let ParcelInstruction {
                robot_id,
                parcel,
                vertex,
            } = instruction;
            new_states[robot_id] = RobotState {
                robot_id,
                parcel_id: Some(parcel),
                vertex: Some(vertex),
            };

            newly_used_vertices.insert(vertex);
        }

        Ok(())
    }
    fn check_for_placement_instruction_error(
        &self,
        instruction: PlacementInstruction,
        used_vertices: &FnvHashMap<Vertex, usize>,
        newly_used_vertices: &mut FnvHashSet<Vertex>,
        new_requests: &mut FnvHashMap<usize, Request>,
    ) -> Option<IllegalPlacementError> {
        let PlacementInstruction {
            robot_id,
            parcel,
            vertex,
        } = instruction;

        if used_vertices.contains_key(&vertex) && used_vertices.get(&vertex) != Some(&robot_id) {
            return Some(IllegalPlacementError::from(
                instruction,
                "Vertex used in previous time step".to_string(),
                self.history.time(),
            ));
        }
        if newly_used_vertices.contains(&vertex) {
            return Some(IllegalPlacementError::from(
                instruction,
                "Vertex already used in next time step".to_string(),
                self.history.time(),
            ));
        }
        if self.history.last_robot_state(robot_id).vertex.unwrap() != vertex {
            return Some(IllegalPlacementError::from(
                instruction,
                "Robot is not at this location".to_string(),
                self.history.time(),
            ));
        }
        if !new_requests.contains_key(&parcel) {
            return Some(IllegalPlacementError::from(
                instruction,
                "Parcel no longer needed".to_string(),
                self.history.time(),
            ));
        }

        None
    }
    fn process_removal_instructions(
        &self,
        removal_instructions: Vec<RemovalInstruction>,
        new_states: &mut Vec<RobotState>,
        new_requests: &mut FnvHashMap<usize, Request>,
        used_vertices: &mut FnvHashSet<Vertex>,
    ) -> Result<(), IllegalRemovalError> {
        for instruction in removal_instructions {
            if self
                .history
                .last_robot_state(instruction.robot_id)
                .vertex
                .is_none()
            {
                continue;
            }
            if let Some(error) = self.check_for_removal_instruction_error(instruction) {
                return Err(error);
            }

            let RemovalInstruction {
                robot_id,
                parcel,
                vertex,
            } = instruction;
            new_states[robot_id] = RobotState {
                robot_id,
                parcel_id: None,
                vertex: Some(vertex),
            };
            new_requests.remove(&parcel);

            used_vertices.insert(self.history.last_robot_state(robot_id).vertex.unwrap());
        }

        Ok(())
    }
    fn check_for_removal_instruction_error(
        &self,
        instruction: RemovalInstruction,
    ) -> Option<IllegalRemovalError> {
        if !self.plan.contains(&instruction.vertex) {
            return Some(IllegalRemovalError::from(
                instruction,
                format!(
                    "Vertex {:?} not part of currently active plan",
                    instruction.vertex,
                ).to_string(),
                self.history.time(),
            ));
        }

        if self
            .history
            .last_robot_state(instruction.robot_id)
            .vertex
            .unwrap() != instruction.vertex
        {
            return Some(IllegalRemovalError::from(
                instruction,
                "Robot is not at this location".to_string(),
                self.history.time(),
            ));
        }

        match self
            .history
            .last_robot_state(instruction.robot_id)
            .parcel_id
        {
            None => {
                return Some(IllegalRemovalError::from(
                    instruction,
                    "Robot has no parcel".to_string(),
                    self.history.time(),
                ))
            }
            Some(parcel) if parcel != instruction.parcel => {
                return Some(IllegalRemovalError::from(
                    instruction,
                    "Robot holds other parcel".to_string(),
                    self.history.time(),
                ));
            }
            _ => (),
        }

        None
    }
    fn process_robot_removal_instructions(
        &self,
        instructions: Vec<RobotRemovalInstruction>,
        new_states: &mut Vec<RobotState>,
    ) {
        for MoveInstruction { robot_id, vertex } in instructions {
            new_states[robot_id].vertex = None;
        }
    }
}
//
//#[cfg(test)]
//mod test {
//
//    use algorithm::assignment::greedy_makespan::GreedyMakespan;
//    use algorithm::path::PathAlgorithm;
//    use algorithm::path::greedy_shortest_paths::GreedyShortestPaths;
//    use simulation::demand::uniform::Uniform;
//    use simulation::plan::one_three_rectangle::OneThreeRectangle;
//    use simulation::demand::Demand;
//    use simulation::settings::Settings;
//    use simulation::simulation::Simulation;
//    use simulation::state::RobotState;
//    use fnv::FnvHashMap;
//    use fnv::FnvHashSet;
//    use simulation::plan::Vertex;
//    use simulation::demand::Request;
//    use simulation::PlacementInstruction;
//    use simulation::state::History;
//    use simulation::state::State;
//    use simulation::MoveInstruction;
//
//    #[test]
//    fn test_process_placement_instructions() {
//        let demand = Box::new(<Uniform as Demand>::create([0; 32]));
//        let plan = OneThreeRectangle::new(3, 3);
//        let settings = Settings {
//            total_time: 6,
//            nr_robots: 2,
//            nr_requests: 2,
//            output_file: None,
//        };
//        let mut assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings),
//        );
//        let mut algorithm = Box::new(GreedyShortestPaths::new(
//            &plan,
//            &settings,
//            assignment_algorithm,
//        ));
//
//        let mut simulation = Simulation::new(algorithm, &plan, demand, &settings);
//        simulation.initialize().ok().unwrap();
//        let mut new_states = vec![
//            RobotState {
//                robot_id: 0,
//                parcel_id: None,
//                vertex: None,
//            },
//            RobotState {
//                robot_id: 1,
//                parcel_id: None,
//                vertex: None,
//            },
//        ];
//        let used_vertices = FnvHashMap::default();
//        let mut newly_used_vertices = FnvHashSet::default();
//        let mut new_requests = FnvHashMap::default();
//        new_requests.insert(
//            0,
//            Request {
//                from: Vertex { x: 0, y: 1 },
//                to: Vertex { x: 2, y: 1 },
//            },
//        );
//        let expected_new_requests = new_requests.clone();
//        assert!(
//            simulation
//                .process_placement_instructions(
//                    vec![PlacementInstruction {
//                        robot_id: 0,
//                        parcel: 0,
//                        vertex: Vertex { x: 0, y: 1 },
//                    }],
//                    &mut new_states,
//                    &used_vertices,
//                    &mut newly_used_vertices,
//                    &mut new_requests,
//                )
//                .is_ok()
//        );
//
//        let expected_new_states = vec![
//            RobotState {
//                robot_id: 0,
//                parcel_id: Some(0),
//                vertex: Vertex { x: 0, y: 1 },
//            },
//            RobotState {
//                robot_id: 1,
//                parcel_id: None,
//                vertex: None,
//            },
//        ];
//        let expected_newly_used_vertices = vec![Vertex { x: 0, y: 1 }].into_iter().collect();
//        assert_eq!(new_states, expected_new_states);
//        assert_eq!(newly_used_vertices, expected_newly_used_vertices);
//        assert_eq!(new_requests, expected_new_requests);
//    }
//
//    #[test]
//    fn test_process_move_instructions_negative() {
//        let demand = Box::new(<Uniform as Demand>::create([0; 32]));
//        let plan = OneThreeRectangle::new(3, 3);
//        let settings = Settings {
//            total_time: 6,
//            nr_robots: 2,
//            nr_requests: 1,
//            output_file: None,
//        };
//
//        let assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
//        let algorithm = Box::new(GreedyShortestPaths::instantiate(
//            &plan,
//            &settings,
//            assignment_algorithm,
//        ));
//
//        let mut simulation = Simulation::new(algorithm, &plan, demand, &settings);
//        simulation.initialize().ok().unwrap();
//        let mut requests = FnvHashMap::default();
//        requests.insert(
//            0,
//            Request {
//                from: Vertex { x: 0, y: 1 },
//                to: Vertex { x: 2, y: 1 },
//            },
//        );
//        requests.insert(
//            1,
//            Request {
//                from: Vertex { x: 1, y: 0 },
//                to: Vertex { x: 1, y: 2 },
//            },
//        );
//        simulation.history = History {
//            states: vec![State {
//                robot_states: vec![
//                    RobotState {
//                        robot_id: 0,
//                        parcel_id: Some(0),
//                        vertex: Vertex { x: 1, y: 1 },
//                    },
//                    RobotState {
//                        robot_id: 1,
//                        parcel_id: Some(1),
//                        vertex: Vertex { x: 1, y: 0 },
//                    },
//                ],
//                requests: requests.clone(),
//            }],
//            calculation_times: Vec::new(),
//        };
//
//        let mut new_states = simulation.history.last_state().robot_states.clone();
//        let mut used_vertices = FnvHashMap::default();
//        used_vertices.insert(Vertex { x: 1, y: 1 }, 0);
//        used_vertices.insert(Vertex { x: 1, y: 0 }, 1);
//        let mut newly_used_vertices = FnvHashSet::default();
//        assert!(
//            simulation
//                .process_move_instructions(
//                    vec![
//                        MoveInstruction {
//                            robot_id: 0,
//                            vertex: Vertex { x: 2, y: 1 },
//                        },
//                        MoveInstruction {
//                            robot_id: 1,
//                            vertex: Vertex { x: 1, y: 1 },
//                        },
//                    ],
//                    &mut new_states,
//                    &used_vertices,
//                    &mut newly_used_vertices,
//                )
//                .is_err()
//        );
//    }
//
//    #[test]
//    fn test_process_move_instructions_positive() {
//        let demand = Box::new(<Uniform as Demand>::create([0; 32]));
//        let plan = OneThreeRectangle::new(3, 3);
//        let settings = Settings {
//            total_time: 6,
//            nr_robots: 2,
//            nr_requests: 1,
//            output_file: None,
//        };
//
//        let assignment_algorithm = Box::new(GreedyMakespan::new(&plan, &settings));
//        let algorithm = Box::new(GreedyShortestPaths::instantiate(
//            &plan,
//            &settings,
//            assignment_algorithm,
//        ));
//
//        let mut simulation = Simulation::new(algorithm, &plan, demand, &settings);
//        simulation.initialize().ok().unwrap();
//        let mut requests = FnvHashMap::default();
//        requests.insert(
//            0,
//            Request {
//                from: Vertex { x: 0, y: 1 },
//                to: Vertex { x: 2, y: 1 },
//            },
//        );
//        requests.insert(
//            1,
//            Request {
//                from: Vertex { x: 1, y: 0 },
//                to: Vertex { x: 1, y: 2 },
//            },
//        );
//        simulation.history = History {
//            states: vec![State {
//                robot_states: vec![
//                    RobotState {
//                        robot_id: 0,
//                        parcel_id: Some(0),
//                        vertex: Vertex { x: 1, y: 1 },
//                    },
//                    RobotState {
//                        robot_id: 1,
//                        parcel_id: Some(1),
//                        vertex: Vertex { x: 0, y: 0 },
//                    },
//                ],
//                requests: requests.clone(),
//            }],
//            calculation_times: Vec::new(),
//        };
//
//        let mut new_states = simulation.history.last_state().robot_states.clone();
//        let mut used_vertices = FnvHashMap::default();
//        used_vertices.insert(Vertex { x: 1, y: 1 }, 0);
//        used_vertices.insert(Vertex { x: 0, y: 0 }, 1);
//        let mut newly_used_vertices = FnvHashSet::default();
//        let new_requests = requests.clone();
//        let expected_new_requests = new_requests.clone();
//        assert!(
//            simulation
//                .process_move_instructions(
//                    vec![
//                        MoveInstruction {
//                            robot_id: 0,
//                            vertex: Vertex { x: 2, y: 1 },
//                        },
//                        MoveInstruction {
//                            robot_id: 1,
//                            vertex: Vertex { x: 0, y: 1 },
//                        },
//                    ],
//                    &mut new_states,
//                    &used_vertices,
//                    &mut newly_used_vertices,
//                )
//                .is_ok()
//        );
//
//        let expected_new_states = vec![
//            RobotState {
//                robot_id: 0,
//                parcel_id: Some(0),
//                vertex: Vertex { x: 2, y: 1 },
//            },
//            RobotState {
//                robot_id: 1,
//                parcel_id: Some(1),
//                vertex: Vertex { x: 0, y: 1 },
//            },
//        ];
//        let expected_newly_used_vertices = vec![Vertex { x: 2, y: 1 }, Vertex { x: 0, y: 1 }]
//            .into_iter()
//            .collect();
//        assert_eq!(new_states, expected_new_states);
//        assert_eq!(newly_used_vertices, expected_newly_used_vertices);
//        assert_eq!(new_requests, expected_new_requests);
//    }
//}
