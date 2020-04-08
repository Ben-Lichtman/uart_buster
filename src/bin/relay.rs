#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;
extern crate panic_semihosting;

use alloc_cortex_m::CortexMHeap;
use cortex_m_rt::{entry, heap_start};

use nrf51_hal::hi_res_timer::TimerFrequency;
use nrf51_hal::prelude::*;
use nrf51_hal::serial::Serial;
use nrf51_hal::serial::BAUDRATE_A;
use nrf51_hal::timer::CountDownTimer;

use nb::block;

fn main() {
	let p = nrf51_hal::nrf51::Peripherals::take().unwrap();

	let uart0 = p.UART0;
	let timer0 = p.TIMER0;
	let gpio = p.GPIO.split();

	// Create timers
	let bitbang_timer = CountDownTimer::new(timer0, TimerFrequency::Freq16MHz);

	// Create host serial port
	let tx_pin = gpio.pin24.into_push_pull_output();
	let rx_pin = gpio.pin25.into_floating_input();
	let (mut host_tx, mut host_rx) =
		Serial::uart0(uart0, tx_pin.into(), rx_pin.into(), BAUDRATE_A::BAUD115200).split();

	// Select pins
	let bang_rx_pin = gpio.pin6.into_floating_input();
	let bang_tx_pin = gpio.pin10.into_push_pull_output();
	// let bang_rx_pin = gpio.pin10.into_floating_input();
	// let bang_tx_pin = gpio.pin6.into_push_pull_output();

	let mut bang_serial = microbit_rust::Serial::new(bang_tx_pin, bang_rx_pin, bitbang_timer);

	bang_serial.set_timeout(750_000);
	bang_serial.set_rate(9600);

	loop {
		match block!(bang_serial.try_read()) {
			Ok(b) => block!(host_tx.write(b)).unwrap(),
			Err(_) => (),
		}

		match host_rx.read() {
			Ok(b) => block!(bang_serial.write(b)).unwrap(),
			Err(_) => (),
		}
	}
}

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[entry]
fn setup() -> ! {
	// Initialize the allocator before you use it
	let start = heap_start() as usize;
	let size = 0x1000; // in bytes
	unsafe { ALLOCATOR.init(start, size) }
	main();
	unreachable!();
}

#[alloc_error_handler]
fn alloc_error(_layout: core::alloc::Layout) -> ! {
	panic!("Out of memory!");
}
