use simulation::plan::Plan;
use simulation::plan::Rectangle;
use simulation::plan::Vertex;

use itertools::Itertools;

pub struct MiddleTerminals {
    pub x_size: u64,
    pub y_size: u64,
    pub padding: u64,
    pub interval: u64,
}

impl MiddleTerminals {
    pub fn new(x_size: u64, y_size: u64, padding: u64, interval: u64) -> MiddleTerminals {
        debug_assert!(x_size > 0);
        debug_assert!(y_size > 0);
        debug_assert!(padding * 2 < x_size);
        debug_assert!(padding * 2 < y_size);
        debug_assert!(interval > 1);

        MiddleTerminals {
            x_size,
            y_size,
            padding,
            interval,
        }
    }
    fn holes(&self) -> Vec<Vertex> {
        let nr_x_terminals = (self.x_size - 2 * self.padding) / self.interval;
        let nr_y_terminals = (self.y_size - 2 * self.padding) / self.interval;

        (0..(nr_x_terminals + 1))
            .cartesian_product(0..(nr_y_terminals + 1))
            .map(|(x, y)| {
                (
                    self.padding + x * self.interval,
                    self.padding + y * self.interval,
                )
            })
            .filter(|&(x, y)| {
                self.padding <= x
                    && x <= self.x_size - self.padding
                    && self.padding <= y
                    && y <= self.y_size - self.padding
            })
            .map(|(x, y)| Vertex { x, y })
            .collect()
    }
}

impl Rectangle for MiddleTerminals {
    fn x_size(&self) -> u64 {
        self.x_size
    }
    fn y_size(&self) -> u64 {
        self.y_size
    }
}

impl Plan for MiddleTerminals {
    fn vertices(&self) -> Vec<Vertex> {
        Rectangle::vertices(self)
            .into_iter()
            .filter(|v| <MiddleTerminals as Plan>::contains(self, v))
            .collect()
    }
    fn contains(&self, &Vertex { x, y }: &Vertex) -> bool {
        x < self.x_size
            && y < self.y_size
            && (x as i64 - self.padding as i64) % self.interval as i64 != 0
            || (y as i64 - self.padding as i64) % self.interval as i64 != 0
    }
    fn sources(&self) -> Vec<Vertex> {
        (0..self.y_size)
            .filter(|&y| self.padding < y && y < self.y_size - self.padding)
            .map(|y| Vertex { x: 0, y })
            .collect()
    }
    fn terminals(&self) -> Vec<Vertex> {
        self.holes()
            .into_iter()
            .flat_map(|v| self.neighbors(&v))
            .collect()
    }
    fn neighbors(&self, &Vertex { x, y }: &Vertex) -> Vec<Vertex> {
        debug_assert!(x < self.x_size);
        debug_assert!(y < self.y_size);

        let mut neighbors = Vec::new();
        if x < self.x_size() - 1 {
            neighbors.push(Vertex { x: x + 1, y });
        }
        if y < self.y_size() - 1 {
            neighbors.push(Vertex { x, y: y + 1 });
        }
        if y > 0 {
            neighbors.push(Vertex { x, y: y - 1 });
        }
        if x > 0 {
            neighbors.push(Vertex { x: x - 1, y });
        }

        neighbors
    }
}
