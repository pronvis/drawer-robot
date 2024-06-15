mod ps3_reader;

pub const CHANNEL_CAPACITY: usize = 32;
pub use ps3_reader::Ps3Reader;

pub enum Ps3Command {
    OnConnect,
    Digital(Ps3DigitalCommand),
    Stick(Ps3StickCommand),
}

#[allow(non_camel_case_types)]
pub enum Ps3DigitalCommand {
    CROSS_DOWN,
    CROSS_UP,
    SQUARE_DOWN,
    SQUARE_UP,
    TRIANGLE_DOWN,
    TRIANGLE_UP,
    CIRCLE_DOWN,
    CIRCLE_UP,
    UP_DOWN,
    UP_UP,
    RIGHT_DOWN,
    RIGHT_UP,
    DOWN_DOWN,
    DOWN_UP,
    LEFT_DOWN,
    LEFT_UP,
    START_DOWN,
    SELECT_DOWN,
    L1_DOWN,
    L2_DOWN,
    R1_DOWN,
    R2_DOWN,
}

impl Ps3DigitalCommand {
    fn from_u8(data: u8) -> Option<Self> {
        match data {
            0x58 => Some(Ps3DigitalCommand::CROSS_DOWN),
            0x78 => Some(Ps3DigitalCommand::CROSS_UP),
            0x53 => Some(Ps3DigitalCommand::SQUARE_DOWN),
            0x73 => Some(Ps3DigitalCommand::SQUARE_UP),
            0x54 => Some(Ps3DigitalCommand::TRIANGLE_DOWN),
            0x74 => Some(Ps3DigitalCommand::TRIANGLE_UP),
            0x43 => Some(Ps3DigitalCommand::CIRCLE_DOWN),
            0x63 => Some(Ps3DigitalCommand::CIRCLE_UP),
            0x55 => Some(Ps3DigitalCommand::UP_DOWN),
            0x75 => Some(Ps3DigitalCommand::UP_UP),
            0x52 => Some(Ps3DigitalCommand::RIGHT_DOWN),
            0x72 => Some(Ps3DigitalCommand::RIGHT_UP),
            0x44 => Some(Ps3DigitalCommand::DOWN_DOWN),
            0x64 => Some(Ps3DigitalCommand::DOWN_UP),
            0x4c => Some(Ps3DigitalCommand::LEFT_DOWN),
            0x6c => Some(Ps3DigitalCommand::LEFT_UP),
            0x57 => Some(Ps3DigitalCommand::START_DOWN),
            0x59 => Some(Ps3DigitalCommand::SELECT_DOWN),
            0x60 => Some(Ps3DigitalCommand::L1_DOWN),
            0x61 => Some(Ps3DigitalCommand::L2_DOWN),
            0x62 => Some(Ps3DigitalCommand::R1_DOWN),
            0x65 => Some(Ps3DigitalCommand::R1_DOWN),
            _ => None,
        }
    }
}

pub struct Ps3StickCommand {
    pub stick: Ps3Stick,
    pub value: Ps3StickValue,
}

#[derive(PartialEq, Eq)]
pub enum Ps3Stick {
    Left,
    Right,
}

pub struct Ps3StickValue {
    pub x_axis: i8,
    pub y_axis: i8,
}
