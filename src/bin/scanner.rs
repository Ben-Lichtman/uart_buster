#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;
extern crate panic_semihosting;

use core::cell::Cell;
use core::mem::MaybeUninit;

use alloc::format;
use alloc_cortex_m::CortexMHeap;
use cortex_m_rt::{entry, heap_start};

use nrf51_hal::gpio::gpio;
use nrf51_hal::hi_res_timer::TimerFrequency;
use nrf51_hal::prelude::*;
use nrf51_hal::serial::Serial;
use nrf51_hal::serial::BAUDRATE_A;
use nrf51_hal::timer::CountDownTimer;

use nb::block;

const BAUD_RATES: &[u64] = &[300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200];
const SEND_DATA: bool = true;

#[allow(dead_code)]
enum PinHolder<T> {
	Pin6(gpio::PIN6<T>),
	Pin10(gpio::PIN10<T>),
	Pin11(gpio::PIN11<T>),
	Pin12(gpio::PIN12<T>),
	Pin18(gpio::PIN18<T>),
}

impl<T> PinHolder<T> {
	fn get_marking(&self) -> u8 {
		match self {
			PinHolder::Pin6(_) => 10,
			PinHolder::Pin10(_) => 9,
			PinHolder::Pin11(_) => 7,
			PinHolder::Pin12(_) => 6,
			PinHolder::Pin18(_) => 8,
		}
	}
}

enum MsgStatus {
	Skipped,
	Got,
	Nothing,
}

fn make_message(
	status: MsgStatus,
	rpin: u8,
	wpin: u8,
	speed: u64,
	host_tx: &mut nrf51_hal::serial::Tx<nrf51_hal::nrf51::UART0>,
) {
	for c in match status {
		MsgStatus::Skipped => format!(
			">   R:{:2} W:{:2} speed:{:6} skipped\r\n",
			rpin, wpin, speed
		),
		MsgStatus::Got => format!("### R:{:2} W:{:2} speed:{:6} got\r\n", rpin, wpin, speed),
		MsgStatus::Nothing => format!(
			"-   R:{:2} W:{:2} speed:{:6} nothing\r\n",
			rpin, wpin, speed
		),
	}
	.bytes()
	{
		block!(host_tx.write(c)).unwrap();
	}
}

fn main() {
	let p = nrf51_hal::nrf51::Peripherals::take().unwrap();

	let uart0 = p.UART0;
	let timer0 = p.TIMER0;
	let gpio = p.GPIO.split();

	// Create timers
	let bitbang_timer = Cell::new(Some(CountDownTimer::new(timer0, TimerFrequency::Freq16MHz)));

	// Create host serial port
	let tx_pin = gpio.pin24.into_push_pull_output();
	let rx_pin = gpio.pin25.into_floating_input();
	let (mut host_tx, mut _host_rx) =
		Serial::uart0(uart0, tx_pin.into(), rx_pin.into(), BAUDRATE_A::BAUD115200).split();

	block!(host_tx.write(b'\r')).unwrap();
	block!(host_tx.write(b'\n')).unwrap();

	let pins = [
		PinHolder::Pin6(gpio.pin6),
		PinHolder::Pin10(gpio.pin10),
		PinHolder::Pin18(gpio.pin18),
		PinHolder::Pin11(gpio.pin11),
		PinHolder::Pin12(gpio.pin12),
	];

	for input_index in 0..pins.len() {
		for output_index in 0..pins.len() {
			// Make sure indices dont match
			if input_index == output_index {
				continue;
			}

			// Unsafe cloning of pin structures

			let input_pin_ptr = &pins[input_index] as *const PinHolder<_>;
			let mut input_pin_clone = MaybeUninit::<PinHolder<_>>::uninit();
			let input_pin_clone = unsafe {
				input_pin_clone
					.as_mut_ptr()
					.copy_from_nonoverlapping(input_pin_ptr, 1);
				input_pin_clone.assume_init()
			};

			let output_pin_ptr = &pins[output_index] as *const PinHolder<_>;
			let mut output_pin_clone = MaybeUninit::<PinHolder<_>>::uninit();
			let output_pin_clone = unsafe {
				output_pin_clone
					.as_mut_ptr()
					.copy_from_nonoverlapping(output_pin_ptr, 1);
				output_pin_clone.assume_init()
			};

			let input_pin_marking = input_pin_clone.get_marking();
			let output_pin_marking = output_pin_clone.get_marking();

			// Convert pins for serial use
			let bang_rx_pin: gpio::PIN<_> = match input_pin_clone {
				PinHolder::Pin6(p) => p.into_floating_input().into(),
				PinHolder::Pin10(p) => p.into_floating_input().into(),
				PinHolder::Pin11(p) => p.into_floating_input().into(),
				PinHolder::Pin12(p) => p.into_floating_input().into(),
				PinHolder::Pin18(p) => p.into_floating_input().into(),
			};
			let bang_tx_pin: gpio::PIN<_> = match output_pin_clone {
				PinHolder::Pin6(p) => p.into_push_pull_output().into(),
				PinHolder::Pin10(p) => p.into_push_pull_output().into(),
				PinHolder::Pin11(p) => p.into_push_pull_output().into(),
				PinHolder::Pin12(p) => p.into_push_pull_output().into(),
				PinHolder::Pin18(p) => p.into_push_pull_output().into(),
			};

			// Create virtual serial port
			let mut bang_serial =
				microbit_rust::Serial::new(bang_tx_pin, bang_rx_pin, bitbang_timer.take().unwrap());

			bang_serial.set_timeout(750_000);

			// Iterate baud rates
			'outer: for speed in BAUD_RATES.iter() {
				bang_serial.set_rate(*speed);

				// Sample many times

				let mut counter = 0;
				let samples = 20;

				for _ in 0..samples {
					if SEND_DATA {
						block!(bang_serial.write(b'A')).unwrap();
					}

					match block!(bang_serial.try_read()) {
						Ok(b) => {
							if b.is_ascii_graphic() {
								counter += 1;
							}
						}
						Err(_) => {
							make_message(
								MsgStatus::Skipped,
								input_pin_marking,
								output_pin_marking,
								*speed,
								&mut host_tx,
							);
							continue 'outer;
						}
					}
				}

				if counter > samples / 2 {
					make_message(
						MsgStatus::Got,
						input_pin_marking,
						output_pin_marking,
						*speed,
						&mut host_tx,
					);
				}
				else {
					make_message(
						MsgStatus::Nothing,
						input_pin_marking,
						output_pin_marking,
						*speed,
						&mut host_tx,
					);
				}
			}

			// Decompose the serial port
			let (_tx, _rx, returned_timer) = bang_serial.destroy();

			bitbang_timer.set(Some(returned_timer));
		}
	}

	for c in "DONE!".bytes() {
		block!(host_tx.write(c)).unwrap();
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
