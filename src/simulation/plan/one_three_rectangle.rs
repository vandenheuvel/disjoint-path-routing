use simulation::plan::Plan;
use simulation::plan::Rectangle;
use simulation::plan::Vertex;

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

        OneThreeRectangle { x_size, y_size }
    }
}
impl Rectangle for OneThreeRectangle {
    fn x_size(&self) -> u64 {
        self.x_size
    }
    fn y_size(&self) -> u64 {
        self.y_size
    }
}
impl Plan for OneThreeRectangle {
    fn vertices(&self) -> Vec<Vertex> {
        Rectangle::vertices(self)
    }
    fn contains(&self, vertex: &Vertex) -> bool {
        Rectangle::contains(self, vertex)
    }
    fn sources(&self) -> Vec<Vertex> {
        (1..self.y_size() - 1).map(|y| Vertex { x: 0, y }).collect()
    }
    fn terminals(&self) -> Vec<Vertex> {
        let mut top = (1..(self.x_size - 1))
            .map(|x| Vertex {
                x,
                y: self.y_size - 1,
            })
            .collect();
        let mut bottom = (1..(self.x_size - 1)).map(|x| Vertex { x, y: 0 }).collect();
        let mut right = (1..(self.y_size - 1))
            .map(|y| Vertex {
                x: self.x_size - 1,
                y,
            })
            .collect();
        let mut terminals = Vec::new();
        terminals.append(&mut top);
        terminals.append(&mut bottom);
        terminals.append(&mut right);

        terminals
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

#[cfg(test)]
mod test {
    use super::*;

    mod vertex {
        use super::*;

        #[test]
        fn distance() {
            let a = Vertex { x: 1, y: 2 };
            let b = Vertex { x: 3, y: 4 };

            assert_eq!(a.distance(b), 4);

            let a = Vertex { x: 3, y: 2 };
            let b = Vertex { x: 1, y: 4 };

            assert_eq!(a.distance(b), 4);
        }
    }

    mod one_three_rectangle {
        use super::*;
        use simulation::plan::Rectangle;
        use std::collections::HashSet;

        fn corners(x_size: u64, y_size: u64) -> [(u64, u64); 4] {
            [
                (0, 0),
                (0, y_size - 1),
                (x_size - 1, 0),
                (x_size - 1, y_size - 1),
            ]
        }

        fn new() -> (u64, u64, OneThreeRectangle) {
            let (x_size, y_size) = (3, 4);
            let plan = OneThreeRectangle::new(x_size, y_size);
            (x_size, y_size, plan)
        }

        #[test]
        fn test_new() {
            new();
        }

        #[test]
        fn test_vertices() {
            let (x_size, y_size, plan) = new();

            // All vertices
            assert_eq!(Rectangle::vertices(&plan).len(), (x_size * y_size) as usize);
            // The lower corner is a vertex
            assert!(<OneThreeRectangle as Plan>::vertices(&plan).contains(&Vertex { x: 0, y: 0 }));
            // The upper corner is a vertex
            assert!(
                <OneThreeRectangle as Plan>::vertices(&plan).contains(&Vertex {
                    x: x_size - 1,
                    y: y_size - 1,
                })
            );
        }

        #[test]
        fn test_sources() {
            let (x_size, y_size, plan) = new();

            // Sources are on the "left" side, `x` coordinate is 0
            // Except for the two corners
            assert_eq!(plan.sources().len(), y_size as usize - 2);
            // Lowest y value source
            assert!(plan.sources().contains(&Vertex { x: 0, y: 1 }));
            // Highest y value source
            assert!(plan.sources().contains(&Vertex {
                x: 0,
                y: y_size - 2,
            }));
            // Corners are not sources
            for &(x, y) in corners(x_size, y_size).iter() {
                assert!(!plan.sources().contains(&Vertex { x, y }));
            }
        }

        #[test]
        fn test_terminals() {
            let (x_size, y_size, plan) = new();

            // Terminals are on the top, bottom and right side
            // Terminals on top, right side and bottom
            assert_eq!(
                plan.terminals().len() as u64,
                (x_size - 2) + (y_size - 2) + (x_size - 2)
            );
            // Top terminal
            assert!(plan.terminals().contains(&Vertex {
                x: 1,
                y: y_size - 1,
            }));
            // Right terminal
            assert!(plan.terminals().contains(&Vertex {
                x: x_size - 1,
                y: y_size - 2,
            }));
            // Bottom terminal
            assert!(plan.terminals().contains(&Vertex { x: 1, y: 0 }));
            // Corners are not terminals
            for &(x, y) in corners(x_size, y_size).iter() {
                assert!(!plan.terminals().contains(&Vertex { x, y }));
            }
        }

        #[test]
        fn test_neighbors() {
            let (x_size, y_size, plan) = new();

            macro_rules! test {
                (($x:expr, $y:expr), [$(($neighbor_x:expr, $neighbor_y:expr)), *]) => {
                    assert_eq!(plan.neighbors(&Vertex { x: $x, y: $y, })
                        .into_iter().collect::<HashSet<_>>(),
                        vec![$(Vertex { x: $neighbor_x, y: $neighbor_y, }), *]
                        .into_iter().collect::<HashSet<_>>());
                }
            }

            // Middle vertex
            test!((1, 1), [(0, 1), (2, 1), (1, 2), (1, 0)]);
            // Edge vertex left
            test!((0, 1), [(0, 2), (0, 0), (1, 1)]);
            // Edge vertex right
            test!(
                (x_size - 1, 1),
                [(x_size - 2, 1), (x_size - 1, 2), (x_size - 1, 0)]
            );
            // Edge vertex top
            test!(
                (1, y_size - 1),
                [(1, y_size - 2), (0, y_size - 1), (2, y_size - 1)]
            );
            // Edge vertex bottom
            test!((1, 0), [(1, 1), (0, 0), (2, 0)]);

            // Corner vertices
            test!((0, 0), [(0, 1), (1, 0)]);
            test!((x_size - 1, 0), [(x_size - 2, 0), (x_size - 1, 1)]);
            test!(
                (x_size - 1, y_size - 1),
                [(x_size - 2, y_size - 1), (x_size - 1, y_size - 2)]
            );
            test!((0, y_size - 1), [(1, y_size - 1), (0, y_size - 2)]);
        }
    }
}
