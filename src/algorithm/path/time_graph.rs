use algorithm::path::greedy_shortest_paths::Path;
use fnv::FnvHashSet;
use priority_queue::PriorityQueue;
use simulation::plan::Plan;
use simulation::plan::Vertex;
use std::cmp::Reverse;
use std::collections::HashMap;
use std::collections::HashSet;
use std::collections::VecDeque;
use std::iter::repeat;

pub struct TimeGraph<'a> {
    plan: &'a Plan,
    vertices: VecDeque<FnvHashSet<Vertex>>,

    earliest_time: usize,
    capacity: usize,
}

impl<'a> TimeGraph<'a> {
    pub fn from_plan(plan: &'a impl Plan, initial_capacity: usize) -> TimeGraph<'a> {
        debug_assert!(plan.sources().len() > 0);
        debug_assert!(plan.terminals().len() > 0);

        let plan_vertices = plan.vertices().into_iter().collect::<FnvHashSet<_>>();
        let vertices = repeat(plan_vertices).take(initial_capacity + 1).collect();

        TimeGraph {
            plan,
            vertices,

            earliest_time: 0,
            capacity: initial_capacity + 1,
        }
    }
    pub fn find_path(&mut self, start_time: usize, from: Vertex, to: Vertex) -> Option<Path> {
        debug_assert_ne!(from, to);

        let start_index = start_time - self.earliest_time;
        if !&self.vertices[start_index].contains(&from) {
            return None;
        }

        let mut came_from = HashMap::new();
        let mut visited = HashSet::new();
        let mut to_visit: PriorityQueue<(usize, Vertex), Reverse<u64>> = PriorityQueue::new();
        to_visit.push((start_index, from), Reverse(from.distance(to)));

        let mut distances: HashMap<TimeVertex, u64> = HashMap::new();
        distances.insert((start_index, from), 0);

        while let Some((current, _)) = to_visit.pop() {
            let (index, vertex) = current;
            if vertex == to {
                if self.vertices[index + 1].contains(&to) {
                    return Some(TimeGraph::reconstruct_path(came_from, current, start_time));
                }
            }

            visited.insert(current);

            for neighbor in self.neighbors(vertex, index) {
                if visited.contains(&neighbor) {
                    continue;
                }

                let new_path_length = distances.get(&current).unwrap() + 1;
                to_visit.push(neighbor, Reverse(new_path_length + neighbor.1.distance(to)));

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
        (previous_index, previous_vertex): TimeVertex,
        start_time: usize,
    ) -> Path {
        debug_assert!(came_from.len() > 0);

        let mut nodes = Vec::new();
        nodes.push(previous_vertex);
        let mut index = previous_index;

        while let Some(&(_, previous_vertex)) = came_from.get(&(index, *nodes.last().unwrap())) {
            index -= 1;
            nodes.push(previous_vertex);
        }
        nodes.reverse();

        let path = Path { start_time, nodes };

        debug_assert!(path.nodes.len() > 1);
        debug_assert!(path.start_time > 0);
        path
    }
    pub fn remove_path(&mut self, path: &Path) {
        debug_assert!(path.nodes.len() > 1);

        let Path { start_time, nodes } = path;
        let start_index = start_time - self.earliest_time;
        for (index, node) in nodes.iter().enumerate() {
            let time = start_index + index;
            if time >= 1 {
                self.vertices[time - 1].remove(node);
            }
            self.vertices[time].remove(node);
            if time + 1 <= self.capacity {
                self.vertices[time + 1].remove(node);
            }
        }
    }
    fn neighbors(&mut self, vertex: Vertex, index: usize) -> Vec<TimeVertex> {
        debug_assert!(index <= self.capacity);

        if index == self.capacity - 1 {
            self.extend(50);
        }

        let mut plan_neighbors = self.plan.neighbors(&vertex);
        plan_neighbors.push(vertex);
        plan_neighbors
            .into_iter()
            .filter(|vertex| self.vertices[index + 1].contains(vertex))
            .map(|vertex| (index + 1, vertex))
            .collect()
    }
    fn extend(&mut self, extra_capacity: usize) {
        let plan_vertices = self.plan.vertices().into_iter().collect::<FnvHashSet<_>>();
        let mut new_layers = repeat(plan_vertices)
            .take(extra_capacity)
            .collect::<VecDeque<_>>();
        self.vertices.append(&mut new_layers);
        self.capacity += extra_capacity;
    }
    pub fn clean_front(&mut self, new_earliest_time: usize) {
        debug_assert!(new_earliest_time > self.earliest_time);

        let to_remove = new_earliest_time - self.earliest_time;
        self.vertices.drain(..to_remove);
        self.capacity -= to_remove;

        self.earliest_time = new_earliest_time;
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
        assert_eq!(
            nr_vertices as u64,
            x_size * y_size * (total_time as u64 + 1)
        );

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
        let mut time_graph = TimeGraph::from_plan(&plan, total_time);

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
        test!(
            (total_time, 1, 1),
            [
                (total_time + 1, 2, 1),
                (total_time + 1, 0, 1),
                (total_time + 1, 1, 2),
                (total_time + 1, 1, 0)
            ]
        );
    }

    #[test]
    fn test_reconstruct_path() {
        let mut came_before = HashMap::new();
        let end = Vertex { x: 3, y: 4 };
        let start = Vertex { x: 2, y: 2 };
        came_before.insert((4, end), (3, Vertex { x: 3, y: 3 }));
        came_before.insert((3, Vertex { x: 3, y: 3 }), (2, Vertex { x: 3, y: 2 }));
        came_before.insert((2, Vertex { x: 3, y: 2 }), (1, start));

        let Path { start_time, nodes } = TimeGraph::reconstruct_path(came_before, (4, end), 1);

        assert_eq!(nodes.first(), Some(&start));
        assert_eq!(nodes.last(), Some(&end));
        assert_eq!(start_time, 1);
    }

    #[test]
    fn test_find_path() {
        let (_, _, total_time, plan) = new();
        let mut time_graph = TimeGraph::from_plan(&plan, total_time);

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

        test!(1, (0, 0), (0, 1), []);
        test!(1, (0, 0), (0, 2), [(0, 1)]);
        test!(1, (0, 0), (2, 0), [(1, 0)]);

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
        let (start_time, total_time) = (1, x_size as usize);
        let mut time_graph = TimeGraph::from_plan(&plan, total_time);
        assert_eq!(
            time_graph.find_path(
                1,
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
    }

    #[test]
    fn test_remove_path() {
        let (_, _, total_time, plan) = new();
        let mut time_graph = TimeGraph::from_plan(&plan, total_time);

        let from = Vertex { x: 0, y: 1 };
        let to = Vertex { x: 1, y: 1 };
        let start_time = 1;
        let path = Path {
            start_time,
            nodes: vec![from, to],
        };
        time_graph.remove_path(&path);

        assert!(!time_graph.vertices[start_time].contains(&from));
        assert!(!time_graph.vertices[start_time + 1].contains(&from));
        assert!(time_graph.vertices[start_time + 2].contains(&from));

        assert!(time_graph.vertices[start_time - 1].contains(&to));
        assert!(!time_graph.vertices[start_time].contains(&to));
        assert!(!time_graph.vertices[start_time + 1].contains(&to));
        assert!(!time_graph.vertices[start_time + 2].contains(&to));
    }
}
