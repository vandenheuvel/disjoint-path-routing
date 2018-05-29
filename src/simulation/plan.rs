

pub trait Plan {
    fn vertices(&self) -> Vec<Vertex> {
        Iterator::zip(0..self.x_size(), 0..self.y_size())
            .map(|(x, y)| Vertex { x, y, })
            .collect()
    }
    fn sources(&self) -> Vec<Vertex>;
    fn terminals(&self) -> Vec<Vertex>;
    fn edges(&self) -> Vec<UndirectedEdge>;
    fn neighbors(&self, vertex: &Vertex) -> Vec<Vertex>;
    fn nr_vertices(&self) -> u64;
    fn x_size(&self) -> u64;
    fn y_size(&self) -> u64;
}

pub struct OneThreeRectangle {
    pub x_size: u64,
    pub y_size: u64,
}

/// Incoming trucks left, outgoing on all three other sides.
/// See Francesco for a picture
impl OneThreeRectangle {
    pub fn new(x_size: u64, y_size: u64) -> OneThreeRectangle {
        debug_assert!(x_size > 0);
        debug_assert!(y_size > 0);

        OneThreeRectangle {
            x_size,
            y_size,
        }
    }
}

impl Plan for OneThreeRectangle {
    fn sources(&self) -> Vec<Vertex> {
        (1..self.y_size()-1)
            .map(|y| Vertex { x: 0, y, })
            .collect()
    }
    fn terminals(&self) -> Vec<Vertex> {
        let mut top = (1..(self.x_size - 1))
            .map(|x| Vertex { x, y: self.y_size - 1, })
            .collect();
        let mut bottom = (1..(self.x_size - 1))
            .map(|x| Vertex { x, y: 0, })
            .collect();
        let mut right = (1..(self.y_size - 1))
            .map(|y| Vertex { x: self.x_size - 1, y, })
            .collect();
        let mut terminals = Vec::new();
        terminals.append(&mut top);
        terminals.append(&mut bottom);
        terminals.append(&mut right);

        terminals
    }
    fn edges(&self) -> Vec<UndirectedEdge> {
        let mut edges = Vec::new();
        
        for Vertex { x, y, } in self.vertices() {
            // Edge up
            if y < self.y_size() - 1 {
                edges.push(UndirectedEdge {
                    first: Vertex { x, y, },
                    second: Vertex { x, y: y + 1, },
                });
            }

            // Edge right
            if x < self.x_size - 1 {
                edges.push(UndirectedEdge {
                    first: Vertex { x, y, },
                    second: Vertex { x: x + 1, y, },
                });
            }
        }

        edges
    }
    fn neighbors(&self, &Vertex { x, y, }: &Vertex) -> Vec<Vertex> {
        debug_assert!(x < self.x_size);
        debug_assert!(y < self.y_size);

        let mut neighbors = Vec::new();
        if x > 0 {
            neighbors.push(Vertex { x: x - 1, y, });
        }
        if x < self.x_size() - 1 {
            neighbors.push(Vertex { x: x + 1, y, });
        }
        if y > 0 {
            neighbors.push(Vertex { x, y: y - 1, });
        }
        if y < self.y_size() - 1 {
            neighbors.push(Vertex { x, y: y + 1, });
        }

        neighbors
    }
    fn nr_vertices(&self) -> u64 {
        self.x_size() * self.y_size()
    }
    fn x_size(&self) -> u64 { self.x_size }
    fn y_size(&self) -> u64 { self.y_size }
}

struct UndirectedEdge {
    first: Vertex,
    second: Vertex,
}
impl PartialEq for UndirectedEdge {
    fn eq(&self, other: &UndirectedEdge) -> bool {
        let &UndirectedEdge { first, second } = other;
        self == &UndirectedEdge { first, second } || self == &UndirectedEdge { second, first }
    }
}

struct DirectedEdge(Vertex, Vertex);
impl From<(Vertex, Vertex)> for DirectedEdge {
    fn from((first, second): (Vertex, Vertex)) -> Self {
        DirectedEdge(first, second)
    }
}
impl Into<(Vertex, Vertex)> for DirectedEdge {
    fn into(self) -> (Vertex, Vertex) {
        let DirectedEdge(from, to) = self;
        (from, to)
    }
}

#[derive(PartialEq, Eq, Copy, Clone, Hash)]
pub struct Vertex {
    x: u64,
    y: u64,
}
impl Vertex {
    pub fn distance(&self, other: Vertex) -> u64 {
        (self.x.max(other.x) - self.x.min(other.x)) + (self.y.max(other.y) - self.y.min(other.y))
    }
}

#[cfg(test)]
mod test {

    use super::*;

    mod vertex {

        use super::*;

        #[test]
        fn distance() {
            let a = Vertex { x: 1, y: 2, };
            let b = Vertex { x: 3, y: 4, };

            assert_eq!(a.distance(b), 4);

            let a = Vertex { x: 3, y: 2, };
            let b = Vertex { x: 1, y: 4, };

            assert_eq!(a.distance(b), 4);
        }
    }
}
