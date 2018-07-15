use std::fs::File;
use std::io;
use std::io::BufWriter;
use std::io::Write;

use itertools::Itertools;
use fnv::FnvHashSet;

pub mod middle_terminals;
pub mod one_three_rectangle;

pub trait Plan: Send + Sync {
    fn vertices(&self) -> Vec<Vertex>;
    fn contains(&self, vertex: &Vertex) -> bool;
    fn sources(&self) -> Vec<Vertex>;
    fn terminals(&self) -> Vec<Vertex>;
    fn neighbors(&self, vertex: &Vertex) -> Vec<Vertex>;
    fn write(&self, writer: &mut BufWriter<File>) -> io::Result<()> {
        writer.write("# Vertices\n".as_bytes())?;
        for Vertex { x, y } in self.vertices() {
            writer.write(format!("{},{}\n", x, y).as_bytes())?;
        }
        writer.write("###\n".as_bytes())?;

        writer.write("# Sources\n".as_bytes())?;
        for Vertex { x, y } in self.sources() {
            writer.write(format!("{},{}\n", x, y).as_bytes())?;
        }
        writer.write("###\n".as_bytes())?;

        writer.write("# Terminals\n".as_bytes())?;
        for Vertex { x, y } in self.terminals() {
            writer.write(format!("{},{}\n", x, y).as_bytes())?;
        }
        writer.write("###\n".as_bytes())?;

        writer.flush()
    }
    fn path_length(&self, from: Vertex, to: Vertex) -> u64 {
        // TODO
        from.distance(to)
    }
    fn neighborhood(&self, vertex: Vertex, radius: u64) -> Vec<Vertex> {
        debug_assert!(self.contains(&vertex));

        let mut discovered = FnvHashSet::default();
        discovered.insert(vertex);
        let mut new_discovered = FnvHashSet::default();
        new_discovered.insert(vertex);

        for _ in 0..radius {
            let mut discovering = FnvHashSet::default();

            for &vertex in new_discovered.iter() {
                for neighbor in self.neighbors(&vertex) {
                    if !discovered.contains(&neighbor) {
                        discovering.insert(neighbor);
                    }
                    discovered.insert(neighbor);
                }
            }

            new_discovered = discovering;
        }

        discovered.into_iter().collect()
    }
}

pub trait Rectangle: Plan {
    fn vertices(&self) -> Vec<Vertex> {
        (0..self.x_size())
            .cartesian_product(0..self.y_size())
            .map(|(x, y)| Vertex { x, y })
            .collect()
    }
    fn contains(&self, vertex: &Vertex) -> bool {
        vertex.x < self.x_size() && vertex.y < self.y_size()
    }
    fn x_size(&self) -> u64;
    fn y_size(&self) -> u64;
}

#[derive(Debug)]
pub struct UndirectedEdge {
    first: Vertex,
    second: Vertex,
}
impl PartialEq for UndirectedEdge {
    fn eq(&self, other: &UndirectedEdge) -> bool {
        let &UndirectedEdge { first, second } = other;
        self.first == first && self.second == second || self.first == second && self.second == first
    }
}

#[derive(Copy, Clone, Debug, Eq, Hash, PartialEq)]
pub struct Vertex {
    pub x: u64,
    pub y: u64,
}
impl Vertex {
    pub fn distance(&self, other: Vertex) -> u64 {
        (self.x.max(other.x) - self.x.min(other.x)) + (self.y.max(other.y) - self.y.min(other.y))
    }
}
