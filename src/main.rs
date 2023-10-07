#![no_main]
#![no_std]
#![allow(unused_imports)]

use panic_halt as _;

use stm32f0xx_hal::{
    delay::Delay,
    pac,
    prelude::*,
};

use cortex_m::peripheral::Peripherals;
use cortex_m_semihosting::debug;
use cortex_m_rt::entry;

// For printing messages in the console using semihosting
use core::fmt::Write;
use cortex_m_semihosting::hio;

// Switch HAL
use switch_hal::{ActiveHigh, OutputSwitch, Switch, IntoSwitch, ToggleableOutputSwitch};

// Quick println implementation
macro_rules! println {
    ($($arg:tt)*) => {
        let _ = writeln!(hio::hstdout().unwrap(), $($arg)*);
    };
}

#[entry]
fn main() -> ! {
    // Get peripherals
    let mut perifs = pac::Peripherals::take().unwrap();
    let core_perifs = Peripherals::take().unwrap();

    // Setup clocks
    let mut rcc = perifs.RCC.configure().sysclk(8.mhz()).freeze(&mut perifs.FLASH);
    let mut delay = Delay::new(core_perifs.SYST, &rcc);

    // Setup GPIO
    let gpioc = perifs.GPIOC.split(&mut rcc);

    // Setup blue and green LEDs
    let (mut blue_led, mut green_led) = cortex_m::interrupt::free(|cs| {
        (
            gpioc.pc8.into_push_pull_output(cs).into_active_high_switch(),
            gpioc.pc9.into_push_pull_output(cs).into_active_high_switch(),
        )
    });

    // Print message
    println!("LEDs setup!");
    blue_led.on().unwrap();
    green_led.off().unwrap();

    // Loop
    loop {
        blue_led.toggle().unwrap();
        green_led.toggle().unwrap();
        delay.delay_ms(500_u16);
    };

}
