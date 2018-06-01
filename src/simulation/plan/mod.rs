use std::io::BufWriter;
use std::io::Write;
use std::fs::File;

use itertools::Itertools;

pub mod one_three_rectangle;

pub trait Plan {
    fn vertices(&self) -> Vec<Vertex>;
    fn sources(&self) -> Vec<Vertex>;
    fn terminals(&self) -> Vec<Vertex>;
    fn edges(&self) -> Vec<UndirectedEdge>;
    fn neighbors(&self, vertex: &Vertex) -> Vec<Vertex>;
    fn write(&self, writer: &mut BufWriter<File>) {
        writer.write("# Vertices\n".as_bytes());
        for Vertex { x, y, } in self.vertices() {
            writer.write(format!("{},{}\n", x, y).as_bytes());
        }
        writer.write("###\n".as_bytes());

        writer.write("# Sources\n".as_bytes());
        for Vertex { x, y, } in self.sources() {
            writer.write(format!("{},{}\n", x, y).as_bytes());
        }
        writer.write("###\n".as_bytes());

        writer.write("# Terminals\n".as_bytes());
        for Vertex { x, y, } in self.terminals() {
            writer.write(format!("{},{}\n", x, y).as_bytes());
        }
        writer.write("###\n".as_bytes());

        writer.flush();
    }
    fn nr_vertices(&self) -> u64;
}

pub trait Rectangle: Plan {
    fn vertices(&self) -> Vec<Vertex> {
        (0..self.x_size())
            .cartesian_product(0..self.y_size())
            .map(|(x, y)| Vertex { x, y })
            .collect()
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
    pub fn distance(&self, other: Vertex) -> i64 {
        ((self.x.max(other.x) - self.x.min(other.x)) + (self.y.max(other.y) - self.y.min(other.y)))
            as i64
    }
}
