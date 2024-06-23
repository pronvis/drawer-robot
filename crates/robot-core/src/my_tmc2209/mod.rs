pub mod communicator;
pub mod configurator;
mod manual_pin;

use communicator::TMC2209SerialCommunicator;
use configurator::TMC2209Configurator;
use rtic_sync::{channel::*, make_channel};

use fugit::Duration;
use stm32f1xx_hal::{
    gpio::{Dynamic, Pin},
    rcc::Clocks,
    timer::{Counter, Event},
};

use tmc2209::{ReadRequest, WriteRequest};

pub const TMC2209COMMUNICATOR_CLOCK_FREQ: u32 = 72_0_000;
const COMMUNICATOR_CHANNEL_CAPACITY: usize = communicator::CHANNEL_CAPACITY;
const TMC2209_BIT_SEND_TICKS: u32 = 40;

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

pub struct Tmc2209Constructor<const PIN_C: char, const PIN_N: u8, TIM>
where
    TIM: stm32f1xx_hal::timer::Instance,
{
    pub communicator: TMC2209SerialCommunicator<PIN_C, PIN_N>,
    pub configurator: TMC2209Configurator,
    pub req_sender: Sender<'static, Request, COMMUNICATOR_CHANNEL_CAPACITY>,
    pub rsp_receiver: Receiver<'static, u32, COMMUNICATOR_CHANNEL_CAPACITY>,
    pub timer: Counter<TIM, TMC2209COMMUNICATOR_CLOCK_FREQ>,
}

impl<const PIN_C: char, const PIN_N: u8, TIM> Tmc2209Constructor<PIN_C, PIN_N, TIM>
where
    TIM: stm32f1xx_hal::timer::Instance,
{
    pub fn new<EnPin>(mut en_pin: EnPin, uart_pin: Pin<PIN_C, PIN_N, Dynamic>, tim: TIM, clocks: &Clocks) -> Self
    where
        EnPin: embedded_hal::digital::v2::OutputPin,
    {
        let _ = en_pin.set_low();

        let mut timer = crate::get_counter(tim, &clocks);
        let tmc2209_timer_ticks = Duration::<u32, 1, TMC2209COMMUNICATOR_CLOCK_FREQ>::from_ticks(TMC2209_BIT_SEND_TICKS);
        timer.start(tmc2209_timer_ticks).unwrap();
        defmt::debug!("tmc2209_communicator_timer period: {} nanos", tmc2209_timer_ticks.to_nanos());
        timer.listen(Event::Update);

        let (tmc2209_req_sender, tmc2209_req_receiver) = make_channel!(Request, COMMUNICATOR_CHANNEL_CAPACITY);
        let (tmc2209_rsp_sender, tmc2209_rsp_receiver) = make_channel!(u32, COMMUNICATOR_CHANNEL_CAPACITY);
        let communicator = TMC2209SerialCommunicator::new(tmc2209_req_receiver, tmc2209_rsp_sender, uart_pin);
        let configurator = TMC2209Configurator::new(tmc2209_req_sender.clone());

        Self {
            communicator,
            configurator,
            req_sender: tmc2209_req_sender,
            rsp_receiver: tmc2209_rsp_receiver,
            timer,
        }
    }
}
