#![no_main]
#![no_std]
#![allow(unused_imports)]

use panic_halt as _;

use stm32f0xx_hal as hal;
use crate::hal::{delay::Delay, pac, prelude::*};

use cortex_m::peripheral::Peripherals;
use cortex_m_semihosting::debug;
use cortex_m_rt::entry;

// For printing messages in the console using semihosting
use core::fmt::Write;
use cortex_m_semihosting::hio;

#[entry]
fn main() -> ! {
    if let (Some(mut p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

        let gpioc = p.GPIOC.split(&mut rcc);

        // (Re-)configure PA1 as output
        let mut blue_led = cortex_m::interrupt::free(move |cs| gpioc.pc8.into_push_pull_output(cs));

        // Get delay provider
        let mut delay = Delay::new(cp.SYST, &rcc);

        loop {
            blue_led.toggle().ok();
            delay.delay_ms(1_000_u16);
            // Write a message in the console
            writeln!(hio::hstdout().unwrap(), "Hello, world!").unwrap();
        }
    }

    loop {
        continue;
    }
}
