use algorithm::greedy_shortest_paths::Path;
use itertools::repeat_n;
use priority_queue::PriorityQueue;
use simulation::plan::Plan;
use simulation::plan::Vertex;
use std::collections::HashMap;
use std::collections::HashSet;
use std::u64;

pub struct TimeGraph<'a> {
    plan: &'a Plan,
    vertices: Vec<HashSet<Vertex>>,
    total_time: usize,
}

impl<'a> TimeGraph<'a> {
    pub fn from_plan(plan: &'a impl Plan, total_time: usize) -> TimeGraph<'a> {
        let plan_vertices = plan.vertices().into_iter().collect::<HashSet<_>>();
        let vertices = repeat_n(plan_vertices, total_time + 1).collect();

        TimeGraph {
            plan,
            vertices,
            total_time,
        }
    }
    pub fn find_earliest_path(&self, from: Vertex, to: Vertex) -> Path {
        for start_time in 0..self.total_time.into() {
            if let Some(path) = self.find_path(start_time.into(), from, to) {
                return path;
            }
        }

        panic!("Time too short");
    }
    fn find_path(&self, start_time: usize, from: Vertex, to: Vertex) -> Option<Path> {
        if !&self.vertices[start_time].contains(&from) {
            return None;
        }

        let mut came_from = HashMap::new();
        let mut visited = HashSet::new();
        let mut to_visit = PriorityQueue::new();
        to_visit.push((start_time, from), -from.distance(to));

        let mut distances: HashMap<TimeVertex, i64> = HashMap::new();
        distances.insert((start_time, from), 0);

        while let Some((current, _)) = to_visit.pop() {
            let (time, vertex) = current;
            if vertex == to {
                return Some(TimeGraph::reconstruct_path(came_from, current));
            }

            visited.insert(current);

            for neighbor in self.neighbors(vertex, time) {
                if visited.contains(&neighbor) {
                    continue;
                }

                let new_path_length = distances.get(&current).unwrap() + 1;
                to_visit.push(neighbor, -(new_path_length + neighbor.1.distance(to)));

                if !distances.contains_key(&neighbor)
                    || new_path_length < *distances.get(&neighbor).unwrap()
                {
                    came_from.insert(neighbor, current);
                    distances.insert(neighbor, new_path_length);
                }
            }
        }

        None
    }
    fn reconstruct_path(
        came_from: HashMap<TimeVertex, TimeVertex>,
        (previous_time, previous_vertex): TimeVertex,
    ) -> Path {
        let mut nodes = Vec::new();
        nodes.push(previous_vertex);
        let mut time = previous_time;

        while let Some(&(_, previous_vertex)) = came_from.get(&(time, *nodes.last().unwrap())) {
            time -= 1;
            nodes.push(previous_vertex);
        }
        nodes.reverse();

        Path {
            start_time: time,
            nodes,
        }
    }
    pub fn remove_path(&mut self, path: &Path) {
        let Path { start_time, nodes } = path;
        for (index, node) in nodes.iter().enumerate() {
            let time = start_time + index;
            self.vertices[time].remove(node);
            if time + 1 < self.total_time {
                self.vertices[time + 1].remove(node);
            }
        }
    }
    fn neighbors(&self, vertex: Vertex, time: usize) -> Vec<TimeVertex> {
        debug_assert!(time <= self.total_time);

        if time < self.total_time {
            self.plan
                .neighbors(&vertex)
                .into_iter()
                .filter(|vertex| self.vertices[time + 1].contains(vertex))
                .map(|vertex| (time + 1, vertex))
                .collect()
        } else {
            Vec::with_capacity(0)
        }
    }
}

type TimeVertex = (usize, Vertex);

#[cfg(test)]
mod test {
    use super::*;
    use simulation::plan::one_three_rectangle::OneThreeRectangle;

    fn new() -> (u64, u64, usize, OneThreeRectangle) {
        let (x_size, y_size, total_time) = (3, 4, 8);
        let plan = OneThreeRectangle::new(x_size, y_size);

        (x_size, y_size, total_time, plan)
    }

    #[test]
    fn test_vertices() {
        let (x_size, y_size, total_time, plan) = new();
        let time_graph = TimeGraph::from_plan(&plan, total_time);

        let nr_vertices = time_graph.vertices.iter().map(HashSet::len).sum::<usize>();
        assert_eq!(nr_vertices as u64, x_size * y_size * total_time as u64);

        let vertices = &time_graph.vertices;
        // Two arbitrary vertices
        for time in 0..total_time {
            assert!(vertices[time].contains(&Vertex { x: 0, y: 0 }));
            assert!(vertices[time].contains(&Vertex {
                x: x_size - 1,
                y: 2,
            }));
        }
        // Vertices are the same for each time
        assert!(
            vertices
                .iter()
                .map(|s| s.len())
                .all(|l| l == vertices[0].len())
        );
        for time in 0..total_time {
            assert_eq!(vertices[time].difference(&vertices[0]).count(), 0);
        }
    }

    #[test]
    fn test_neighbors() {
        let (_, _, total_time, plan) = new();
        let time_graph = TimeGraph::from_plan(&plan, total_time);

        macro_rules! test {
            (($t: expr, $x:expr, $y:expr),
             [$(($neighbor_t:expr, $neighbor_x:expr, $neighbor_y:expr)), *]
            ) => {
                assert_eq!(time_graph.neighbors(Vertex { x: $x, y: $y, }, $t)
                    .into_iter().collect::<HashSet<_>>(),
                    vec![$(($neighbor_t, Vertex { x: $neighbor_x, y: $neighbor_y, })), *]
                    .into_iter().collect::<HashSet<_>>());
            }
        }

        // Middle
        test!((0, 1, 1), [(1, 2, 1), (1, 0, 1), (1, 1, 2), (1, 1, 0)]);
        // Boundary
        test!((3, 0, 1), [(4, 1, 1), (4, 0, 0), (4, 0, 2)]);
        // Corner
        test!((3, 0, 0), [(4, 1, 0), (4, 0, 1)]);
        // Final time period
        test!((total_time, 0, 0), []);
    }

    #[test]
    fn test_reconstruct_path() {
        let mut came_before = HashMap::new();
        let end = Vertex { x: 3, y: 4 };
        let start = Vertex { x: 2, y: 2 };
        came_before.insert((4, end), (3, Vertex { x: 3, y: 3 }));
        came_before.insert((3, Vertex { x: 3, y: 3 }), (2, Vertex { x: 3, y: 2 }));
        came_before.insert((2, Vertex { x: 3, y: 2 }), (1, start));

        let Path { start_time, nodes } = TimeGraph::reconstruct_path(came_before, (4, end));

        assert_eq!(nodes.first(), Some(&start));
        assert_eq!(nodes.last(), Some(&end));
        assert_eq!(start_time, 1);
    }

    #[test]
    fn test_find_path() {
        let (_, _, total_time, plan) = new();
        let time_graph = TimeGraph::from_plan(&plan, total_time);

        macro_rules! test {
            ($start_time:expr,
             ($from_x:expr, $from_y:expr),
             ($to_x:expr, $to_y:expr),
             [$(($path_x:expr, $path_y:expr)), *]
            ) => {
                let from = Vertex { x: $from_x, y: $from_y, };
                let to = Vertex { x: $to_x, y: $to_y, };
                let path = time_graph.find_path($start_time, from, to);
                assert_eq!(path, Some(Path { start_time: $start_time,
                                             nodes: vec![from,
                                                         $(Vertex { x: $path_x, y: $path_y, }, )*
                                                         to],}));
            }
        }

        test!(0, (0, 0), (0, 1), []);
        test!(1, (0, 0), (0, 2), [(0, 1)]);
        test!(0, (0, 0), (2, 0), [(1, 0)]);

        // Two shortest paths
        let start_time = 2;
        let from = Vertex { x: 0, y: 0 };
        let to = Vertex { x: 1, y: 1 };
        let path = time_graph.find_path(start_time, from, to);
        assert!(
            path == Some(Path {
                start_time,
                nodes: vec![from, Vertex { x: 1, y: 0 }, to],
            }) || path == Some(Path {
                start_time,
                nodes: vec![from, Vertex { x: 0, y: 1 }, to],
            })
        );

        let (x_size, y_size) = (2, 3);
        let plan = OneThreeRectangle::new(x_size, y_size);
        let from = Vertex { x: 0, y: 1 };
        let to = Vertex {
            x: x_size - 1,
            y: 1,
        };
        // Time just long enough
        let (start_time, total_time) = (0, x_size as usize - 1);
        let time_graph = TimeGraph::from_plan(&plan, total_time);
        assert_eq!(
            time_graph.find_path(
                0,
                Vertex { x: 0, y: 1 },
                Vertex {
                    x: x_size - 1,
                    y: 1,
                }
            ),
            Some(Path {
                start_time,
                nodes: vec![from, to],
            })
        );
        // Time just too short
        let total_time = x_size as usize - 2;
        let time_graph = TimeGraph::from_plan(&plan, total_time);
        assert_eq!(
            time_graph.find_path(
                0,
                Vertex { x: 0, y: 1 },
                Vertex {
                    x: x_size - 1,
                    y: 1,
                }
            ),
            None
        );
    }

    #[test]
    fn test_find_earliest_path() {
        let (_, _, total_time, plan) = new();
        let time_graph = TimeGraph::from_plan(&plan, total_time);
        let from = Vertex { x: 0, y: 1 };
        let to = Vertex { x: 1, y: 1 };

        assert_eq!(
            time_graph.find_earliest_path(Vertex { x: 0, y: 1 }, Vertex { x: 1, y: 1 }),
            Path {
                start_time: 0,
                nodes: vec![from, to],
            }
        );
    }

    #[test]
    fn test_remove_path() {
        let (_, _, total_time, plan) = new();
        let mut time_graph = TimeGraph::from_plan(&plan, total_time);

        let from = Vertex { x: 0, y: 1 };
        let to = Vertex { x: 1, y: 1 };
        let start_time = 0;
        let path = Path {
            start_time,
            nodes: vec![from, to],
        };
        time_graph.remove_path(&path);

        assert!(!time_graph.vertices[start_time].contains(&from));
        assert!(!time_graph.vertices[start_time + 1].contains(&from));
        assert!(!time_graph.vertices[start_time + 1].contains(&to));
        assert!(!time_graph.vertices[start_time + 2].contains(&to));
    }
}
