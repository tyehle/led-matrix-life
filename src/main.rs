#![no_std]
#![no_main]

extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use feather_m0 as hal;

use atsamd21g18a::{TC4, TC5};
use cortex_m_rt::entry;
use hal::clock::GenericClockController;
use hal::gpio::*;
use hal::pac::Peripherals;
use hal::prelude::*;
use hal::sercom::{SPIMaster4, Sercom4Pad0, Sercom4Pad2, Sercom4Pad3};
use hal::timer::TimerCounter;
use nb::block;

use matrix_display::*;

type SPI = SPIMaster4<Sercom4Pad0<Pa12<PfD>>, Sercom4Pad2<Pb10<PfD>>, Sercom4Pad3<Pb11<PfD>>>;
type LEDPin = Pa17<Output<OpenDrain>>;

/// Delay struct compatible with both the feather m0 timer and the LED Matrix
#[derive(Clone, Copy)]
struct DelayHertz(u32);
impl From<DelayHertz> for hal::time::Hertz {
    fn from(delay: DelayHertz) -> hal::time::Hertz {
        hal::time::Hertz(delay.0)
    }
}

impl core::ops::Shl<usize> for DelayHertz {
    type Output = DelayHertz;
    fn shl(self, amount: usize) -> DelayHertz {
        DelayHertz(self.0 << amount)
    }
}

/// Get the SPI bus setup
fn setup() -> (
    LEDPin,
    TimerCounter<TC5>,
    LEDArray<
        Pa7<Output<OpenDrain>>,
        Pa18<Output<OpenDrain>>,
        Pa16<Output<OpenDrain>>,
        TimerCounter<TC4>,
        SPI,
        Pa20<Output<OpenDrain>>,
        Pa15<Output<OpenDrain>>,
    >,
) {
    let mut peripherals = Peripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let mut pins = hal::Pins::new(peripherals.PORT);

    // initialize the pins
    let row_pins = (
        pins.d9.into_open_drain_output(&mut pins.port),
        pins.d10.into_open_drain_output(&mut pins.port),
        pins.d11.into_open_drain_output(&mut pins.port),
    );

    let reg_pin = pins.d6.into_open_drain_output(&mut pins.port);
    let output_disable = pins.d5.into_open_drain_output(&mut pins.port);

    let mut red_led = pins.d13.into_open_drain_output(&mut pins.port);
    red_led.set_low().unwrap();

    // Setup the timer
    let gclk0 = clocks.gclk0();
    let tc45 = &clocks.tc4_tc5(&gclk0).unwrap();
    let timer = hal::timer::TimerCounter::tc4_(tc45, peripherals.TC4, &mut peripherals.PM);

    let tc5 = hal::timer::TimerCounter::tc5_(tc45, peripherals.TC5, &mut peripherals.PM);

    // setup the SPI bus
    let spi = hal::spi_master(
        &mut clocks,
        10.mhz(),
        peripherals.SERCOM4,
        &mut peripherals.PM,
        pins.sck,
        pins.mosi,
        pins.miso,
        &mut pins.port,
    );

    let mut array = LEDArray {
        // array: image,
        array: [[0; 16]; 8],
        row_pins,
        timer,
        spi,
        reg_pin,
        output_disable,
    };
    // start the timer so we don't crash on the first scan
    array.timer.start(1.mhz());

    (red_led, tc5, array)
}

fn count_neighbors_bounded(state: &[[u8; 16]; 8], row: usize, col: usize) -> u8 {
    let mut total = 0;
    for r in row.saturating_sub(1)..=core::cmp::min(7, row+1) {
        for c in col.saturating_sub(1)..=core::cmp::min(15, col+1) {
            if r == row && c == col {
                continue;
            }
            total += state[r][c] & 1;
        }
    }
    total
}

fn count_neighbors_torus(state: &[[u8; 16]; 8], row: usize, col: usize) -> u8 {
    let mut total = 0;
    for roff in 7..=9 {
        for coff in 15..=17 {
            if roff == 8 && coff == 16 {
                continue;
            }

            let r = (row + roff) % 8;
            let c = (col + coff) % 16;
            total += state[r][c] & 1;
        }
    }
    total
}

fn step_state(state: &mut [[u8; 16]; 8]) {
    // we can't allocate, so use the second lowest bit to signify what will
    // happen in the next iteration
    for row in 0..8 {
        for col in 0..16 {
            let neighbors = count_neighbors_torus(&state, row, col);
            if state[row][col] & 1 == 0 && neighbors == 3 {
                // we are dead and have 3 live neighbors
                state[row][col] |= 0b10;
            } else if state[row][col] & 1 == 1 && (neighbors == 2 || neighbors == 3) {
                // we are alive and have 2 or 3 live neighbors
                state[row][col] |= 0b10;
            } else {
                // we should die
                // the next bit is already 0
            }
        }
    }

    // shift all the bits down one
    for row in 0..8 {
        for col in 0..16 {
            state[row][col] = state[row][col] >> 1;
        }
    }
}

fn show_state(state: &[[u8; 16]; 8], image: &mut [[u8; 16]; 8]) {
    for row in 0..8 {
        for col in 0..16 {
            image[row][col] = if state[row][col] == 1 {15} else {0};
        }
    }
}

#[entry]
fn main() -> ! {
    let (mut red_led, mut _timer, mut array) = setup();

    let mut state = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ];

    let base_scan_freq = DelayHertz(1000);

    show_state(&state, &mut array.array);

    let frame_duration = 8;
    let mut frame_timeout = 100;

    loop {
        if frame_timeout == 0 {
            show_state(&state, &mut array.array);
            step_state(&mut state);
            frame_timeout = frame_duration;
        }
        frame_timeout -= 1;
        array.scan(base_scan_freq).unwrap_or(());
        red_led.toggle();
    }
}
