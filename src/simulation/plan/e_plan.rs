use simulation::plan::Plan;
use simulation::plan::Rectangle;
use simulation::plan::Vertex;

pub struct EPlan {
    pub x_size: u64,
    pub y_size: u64,
    pub vertical_piece_width: u64,
    pub horizontal_piece_height: u64,
}

impl EPlan {
    pub fn new(x_size: u64, y_size: u64, vertical_piece_width: u64, horizontal_piece_height: u64) -> EPlan {
        assert_eq!((y_size - horizontal_piece_height) % (horizontal_piece_height + 2), 0);

        EPlan {
            x_size,
            y_size,
            vertical_piece_width,
            horizontal_piece_height,
        }
    }
    fn nr_gaps(&self) -> u64 {
        (self.y_size - self.horizontal_piece_height) / (self.horizontal_piece_height + 2) + 1
    }
}

impl Rectangle for EPlan {
    fn x_size(&self) -> u64 {
        self.x_size
    }

    fn y_size(&self) -> u64 {
        self.y_size
    }
}

impl Plan for EPlan {
    fn vertices(&self) -> Vec<Vertex> {
        Rectangle::vertices(self)
            .into_iter()
            .filter(|v| <EPlan as Plan>::contains(self, v))
            .collect()
    }

    fn contains(&self, &Vertex { x, y }: &Vertex) -> bool {
        if x < self.x_size && y < self.y_size {
            if x < self.vertical_piece_width { true } else {
                if y < self.nr_gaps() * (self.horizontal_piece_height + 2) {
                    y % (self.horizontal_piece_height + 2) < 3
                } else { true }
            }
        } else { false }
    }

    fn sources(&self) -> Vec<Vertex> {
        (0..self.y_size)
            .map(|y| Vertex { x: 0, y, })
            .collect()
    }

    fn terminals(&self) -> Vec<Vertex> {
        let mut below = (self.vertical_piece_width..self.x_size)
            .map(|x| Vertex { x, y: 0, })
            .collect::<Vec<_>>();

        let mut middle_tops = Vec::new();
        for i in 0..(self.nr_gaps() - 1) {
            let y = self.horizontal_piece_height - 1 + i * (self.horizontal_piece_height + 2);
            let mut row = (self.vertical_piece_width..self.x_size)
                .map(|x| Vertex { x, y, })
                .collect::<Vec<_>>();
            middle_tops.append(&mut row);
        }
        let mut middle_bottoms = Vec::new();
        for i in 1..self.nr_gaps() {
            let y = i * (self.horizontal_piece_height + 2);
            let mut row = (self.vertical_piece_width..self.x_size)
                .map(|x| Vertex { x, y, })
                .collect::<Vec<_>>();
            middle_bottoms.append(&mut row);
        }

        let mut top = (self.vertical_piece_width..self.x_size)
            .map(|x| Vertex { x, y: self.y_size - 1, })
            .collect::<Vec<_>>();

        let mut vertices = Vec::with_capacity(
            below.len() + middle_tops.len() + middle_bottoms.len() + top.len()
        );
        vertices.append(&mut below);
        vertices.append(&mut middle_tops);
        vertices.append(&mut middle_bottoms);
        vertices.append(&mut top);

        vertices
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
            .into_iter()
            .filter(|v| <EPlan as Plan>::contains(self, v))
            .collect()
    }
}