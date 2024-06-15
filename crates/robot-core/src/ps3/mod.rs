mod ps3_parser;
mod ps3_reader;

pub const CHANNEL_CAPACITY: usize = 32;
pub use ps3_parser::Ps3EventParser;
pub use ps3_reader::{Ps3Event, Ps3Reader};
