use std::fs::File;
use std::io;
use std::io::BufWriter;
use std::io::Write;

use fnv::FnvHashSet;
use itertools::Itertools;
use std::hash::Hash;
use std::hash::Hasher;
use std::num::Wrapping;
use fnv::FnvHasher;

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
    fn edges(&self) -> FnvHashSet<UndirectedEdge> {
        let mut edges = FnvHashSet::default();

        let vertices = self.vertices();
        for v1 in vertices.iter() {
            for v2 in vertices.iter() {
                if v1.distance(*v2) == 1 {
                    edges.insert(UndirectedEdge {
                        first: *v1,
                        second: *v2,
                    });
                }
            }
        }

        edges
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

#[derive(Debug, Eq)]
pub struct UndirectedEdge {
    pub first: Vertex,
    pub second: Vertex,
}
impl PartialEq for UndirectedEdge {
    fn eq(&self, other: &UndirectedEdge) -> bool {
        let &UndirectedEdge { first, second } = other;
        self.first == first && self.second == second || self.first == second && self.second == first
    }
}
impl Hash for UndirectedEdge {
    fn hash<H: Hasher>(&self, state: &mut H) {
        let mut first_hasher = FnvHasher::with_key(0);
        self.first.hash(&mut first_hasher);
        let first_hash = first_hasher.finish();

        let mut second_hasher = FnvHasher::with_key(0);
        self.second.hash(&mut second_hasher);
        let second_hash = second_hasher.finish();

        let first_wrapped = Wrapping(first_hash);
        let second_wrapped = Wrapping(second_hash);
        let result = first_wrapped + second_wrapped;
        result.hash(state);
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
