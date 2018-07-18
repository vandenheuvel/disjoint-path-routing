#![feature(nll)]
#![feature(macro_at_most_once_rep)]

extern crate core;
extern crate fnv;
extern crate itertools;
extern crate priority_queue;
extern crate rand;

#[macro_use]
pub mod macros;

pub mod algorithm;
pub mod simulation;

#[cfg(test)]
mod test;
