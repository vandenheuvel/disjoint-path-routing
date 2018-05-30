use std::u64;

use simulation::plan::Plan;
use simulation::plan::Vertex;
use std::collections::HashSet;
use algorithm::greedy_shortest_paths::Path;
use priority_queue::PriorityQueue;
use std::collections::HashMap;

pub struct TimeGraph<'a> {
    plan: &'a Plan,
    vertices: Vec<HashSet<Vertex>>,
    total_time: usize,
}

impl<'a> TimeGraph<'a> {
    pub fn from_plan(plan: &'a Plan, total_time: usize) -> TimeGraph<'a> {
        let mut vertices = Vec::with_capacity(total_time.into());
        for _ in 0..total_time.into() {
            vertices.push(plan.vertices().into_iter().collect());
        }

        TimeGraph { plan, vertices, total_time, }
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
        if !&self.vertices[<usize as Into<usize>>::into(start_time)].contains(&from) { return None; }

        let mut came_from = HashMap::new();
        let mut visited = HashSet::new();
        let mut to_visit = PriorityQueue::new();
        to_visit.push((start_time, from), TimeGraph::distance(from, to));

        let mut distances = HashMap::new();

        while let Some((current, path_length)) = to_visit.pop() {
            let (time, vertex) = current;
            if vertex == to {
                return Some(TimeGraph::reconstruct_path(came_from, current));
            }

            visited.insert(current);

            for neighbor in self.neighbors(vertex, time) {
                if visited.contains(&neighbor) {
                    continue;
                }

                to_visit.push(neighbor, path_length + 1 + TimeGraph::distance(neighbor.1, to));

                let new_path_length = path_length + 1;
                if !distances.contains_key(&neighbor)
                    || new_path_length < *distances.get(&neighbor).unwrap() {
                    came_from.insert(neighbor, current);
                    distances.insert(neighbor, new_path_length);
                }
            }
        }

        None
    }
    fn reconstruct_path(came_from: HashMap<TimeVertex, TimeVertex>, (previous_time, previous_vertex): TimeVertex) -> Path {
        let mut nodes = Vec::new();
        nodes.push(previous_vertex);
        let mut time = previous_time;

        while let Some(&(_, previous_vertex)) = came_from.get(&(time, *nodes.last().unwrap())) {
            time -= 1;
            nodes.push(previous_vertex);
        }
        nodes.reverse();

        Path { started: false, start_time: time, nodes, }
    }
    fn distance(first: Vertex, second: Vertex) -> u64 {
        u64::max_value() - first.distance(second)
    }
    pub fn remove_path(&mut self, path: &Path) {
        let Path { started, start_time, nodes, } = path;
        for (index, node) in nodes.iter().enumerate() {
            self.vertices[start_time + index].remove(node);
            self.vertices[start_time + index + 1].remove(node);
        }
    }
    fn neighbors(&self, vertex: Vertex, time: usize) -> Vec<TimeVertex> {
        debug_assert!(time < self.total_time);
        
        self.plan.neighbors(&vertex).into_iter()
            .filter(|vertex| self.vertices[time + 1].contains(vertex))
            .map(|vertex| (time + 1, vertex))
            .collect()
    }
}

type TimeVertex = (usize, Vertex);

#[cfg(test)]
mod test {

    use super::*;
    use simulation::plan::OneThreeRectangle;

    #[test]
    fn from_plan() {
        let (x_size, y_size, total_time) = (2, 3, 4);
        let plan = OneThreeRectangle::new(x_size, y_size);
        let graph = TimeGraph::from_plan(&plan, total_time);

        let nr_vertices = graph.vertices.iter().map(HashSet::len).sum::<usize>();
        assert_eq!(nr_vertices as u64, x_size * y_size * total_time as u64);

    }

}