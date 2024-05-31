pub mod communicator;
pub mod configurator;

use tmc2209::{ReadRequest, WriteRequest};

pub enum Request {
    Write(WriteRequest),
    Read(ReadRequest),
}

impl Request {
    pub fn write(req: WriteRequest) -> Self {
        return Request::Write(req);
    }

    pub fn read(req: ReadRequest) -> Self {
        return Request::Read(req);
    }
}
