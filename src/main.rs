#![no_main]
#![no_std]
#![allow(unused_imports)]

use panic_halt as _;

use stm32f0xx_hal as hal;
use crate::hal::{
    delay::Delay,
    gpio::*,
    pac::{interrupt, Interrupt, Peripherals, EXTI, syscfg},
    prelude::*,
    serial::{self, Serial},
    pwm::{self, PwmChannels},
};

use core::{cell::RefCell, ops::DerefMut, fmt::Write};

use cortex_m::{interrupt::Mutex, peripheral::Peripherals as c_m_Peripherals, peripheral::syst::SystClkSource::Core};
use cortex_m_semihosting::hio;
use cortex_m_rt::{entry, exception};

// Make our LED globally available
static BLUE_LED: Mutex<RefCell<Option<gpioc::PC8<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static GREEN_LED: Mutex<RefCell<Option<gpioc::PC9<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static SYSTICK_OUT: Mutex<RefCell<Option<gpioc::PC7<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

// USART2
static USART2: Mutex<RefCell<Option<Serial<stm32f0xx_hal::pac::USART2, gpioa::PA2<Alternate<AF1>>, gpioa::PA3<Alternate<AF1>>>>>> = Mutex::new(RefCell::new(None));

// Make our delay provider globally available
static DELAY: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));

// Make external interrupt registers globally available
static INT: Mutex<RefCell<Option<EXTI>>> = Mutex::new(RefCell::new(None));

// Create global systick counter (with mutex)
static SYSTICK_COUNT: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(Some(0)));

// PWM (from let mut pwm = pwm::tim1(p.TIM1, pa11, &mut rcc, 1.khz());)
static PWM: Mutex<RefCell<Option<PwmChannels<stm32f0xx_hal::pac::TIM1, stm32f0xx_hal::pwm::C4>>>> = Mutex::new(RefCell::new(None));

// Semihosted println implementation
// WARNING: This is very slow and will block the MCU
macro_rules! println {
    ($($arg:tt)*) => {
        let _ = writeln!(hio::hstdout().unwrap(), $($arg)*);
    };
}

#[entry]
fn main() -> ! {
    // Execute all setup in a critical section
    if let (Some(p), Some(cp)) = (Peripherals::take(), c_m_Peripherals::take()) {
        cortex_m::interrupt::free(move |cs| {
            // Enable clock for SYSCFG
            let rcc = p.RCC;
            rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

            let mut flash = p.FLASH;
            // let mut rcc = rcc.configure().sysclk(8.mhz()).freeze(&mut flash);
            // Instead of the internal 8MHz RC oscillator, we will use the external 8MHz crystal on the discovery board
            // HSE Bypassmode is not used, as a crystal is directly connected to the OSC_IN and OSC_OUT pins
            let mut rcc = rcc.configure().hse(8.mhz(), hal::rcc::HSEBypassMode::NotBypassed).freeze(&mut flash);

            // Get other system info and print
            let (hclk_mhz, pclk_mhz, sysclk_mhz) = 
            (
                rcc.clocks.hclk().0 / 1_000_000, 
                rcc.clocks.pclk().0 / 1_000_000, 
                rcc.clocks.sysclk().0 / 1_000_000
            );
            println!("SYSCLK: {} Hz\nHCLK: {} Hz\nPCLK: {} Hz", sysclk_mhz, hclk_mhz, pclk_mhz);

            // Get CPUID 
            unsafe {
                let cpuid: u32 = cortex_m::peripheral::CPUID::PTR.read().base.read();
                let implementer = ((cpuid & 0xff00_0000) >> 24) as u8;
                // Decode implementer to str
                let implementer_str = match implementer {
                    0x41 => "ARM",
                    _ => "Other"
                };
                let variant = ((cpuid & 0x00f0_0000) >> 20) as u8;
                let architecture = ((cpuid & 0x000f_0000) >> 16) as u8;
                // Decode architecture to str
                let architecture_str = match architecture {
                    0xC => "ARMv6-M",
                    _ => "Other"
                };
                let partno = ((cpuid & 0x0000_fff0) >> 4) as u16;
                // Decode partno to str
                let partno_str = match partno {
                    0xC20 => "Cortex-M0",
                    0xC60 => "Cortex-M0+",
                    0xC21 => "Cortex-M1",
                    0xC23 => "Cortex-M3",
                    0xC24 => "Cortex-M4",
                    0xC27 => "Cortex-M7",
                    _ => "Other"
                };
                let revision = (cpuid & 0x0000_000f) as u8;
                println!("Implementer: {}\nVariant: {}\nArchitecture: {}\nPartno: {}\nRevision: {}", implementer_str, variant, architecture_str, partno_str, revision);            }

            let gpioa = p.GPIOA.split(&mut rcc);
            let gpioc = p.GPIOC.split(&mut rcc);
            let syscfg = p.SYSCFG;
            let exti = p.EXTI;

            let mut syst = cp.SYST;

            let _ = gpioa.pa0.into_pull_down_input(cs);

            let mut blue_led = gpioc.pc8.into_push_pull_output(cs);
            let mut green_led = gpioc.pc9.into_push_pull_output(cs);

            let systick_out = gpioc.pc7.into_push_pull_output(cs);

            // Init USART 2 (PA3 - RX, PA2 - TX)
            let tx = gpioa.pa2.into_alternate_af1(cs);
            let rx = gpioa.pa3.into_alternate_af1(cs);
            
            // expected struct `Serial<USART2, (PA2<Alternate<AF1>>, PA3<Alternate<AF1>>), u32>` 
            let usart2 = Serial::usart2(
                p.USART2,
                (tx, rx),
                115_200.bps(),
                &mut rcc,
            );

            // PWM setup
            let pa11 = gpioa.pa11.into_alternate_af2(cs);
            let mut pwm = pwm::tim1(p.TIM1, pa11, &mut rcc, 1.khz());
            let max_duty = pwm.get_max_duty();
            pwm.set_duty(max_duty / 2);
            pwm.enable();

            // Turn off LED
            blue_led.set_low().unwrap();
            green_led.set_low().unwrap();

            // Enable external interrupt
            syscfg.exticr1.modify(|_, w| unsafe { w.exti0().bits(0) });

            // Set interrupt request mask
            exti.imr.modify(|_, w| w.mr0().set_bit());

            // Set interrupt falling trigger
            exti.ftsr.modify(|_, w| w.tr0().set_bit());

            // Initialise SysTick counter with a defined value
            unsafe { syst.cvr.write(1) };

            // Set source for SysTick counter, here full operating frequency (== 48MHz)
            syst.set_clock_source(Core);

            // Make it so it outputs the clock on SystickOut
            syst.set_reload(rcc.clocks.sysclk().0 - 1);

            // Start SysTick counter
            syst.enable_counter();

            // Start SysTick interrupt generation
            syst.enable_interrupt();
            
            // Initialise delay provider
            let delay = Delay::new(syst, &rcc);

            // Move control over LED and DELAY and EXTI into global mutexes
            *BLUE_LED.borrow(cs).borrow_mut() = Some(blue_led);
            *GREEN_LED.borrow(cs).borrow_mut() = Some(green_led);
            *SYSTICK_OUT.borrow(cs).borrow_mut() = Some(systick_out);
            *PWM.borrow(cs).borrow_mut() = Some(pwm);
            *USART2.borrow(cs).borrow_mut() = Some(usart2);
            *DELAY.borrow(cs).borrow_mut() = Some(delay);
            *INT.borrow(cs).borrow_mut() = Some(exti);

            // Enable EXTI IRQ, set prio 1 and clear any pending IRQs
            let mut nvic = cp.NVIC;
            unsafe {
                nvic.set_priority(Interrupt::EXTI0_1, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI0_1);
            }
            cortex_m::peripheral::NVIC::unpend(Interrupt::EXTI0_1);
        });

    }

    loop {
        continue;
    }

}

// Define an interupt handler, i.e. function to call when interrupt occurs. Here if our external
// interrupt trips when the button is pressed and will light the LED for a second
#[interrupt]
fn EXTI0_1() {
    // Enter critical section
    cortex_m::interrupt::free(|cs| {
        // Obtain all Mutex protected resources
        if let (
            &mut Some(ref mut blue_led), 
            &mut Some(ref mut green_led), 
            &mut Some(ref mut delay), 
            &mut Some(ref mut exti), 
            &mut Some(ref mut systick_count),
            &mut Some(ref mut pwm),
        ) = (
            BLUE_LED.borrow(cs).borrow_mut().deref_mut(),
            GREEN_LED.borrow(cs).borrow_mut().deref_mut(),
            DELAY.borrow(cs).borrow_mut().deref_mut(),
            INT.borrow(cs).borrow_mut().deref_mut(),
            SYSTICK_COUNT.borrow(cs).borrow_mut().deref_mut(),
            PWM.borrow(cs).borrow_mut().deref_mut(),
        ) {
            println!("Button pressed");
            // Turn on LED
            // if count is even, toggle blue, else toggle green
            if *systick_count % 2 == 0 {
                blue_led.toggle().ok();
                pwm.disable();
            } else {
                green_led.toggle().ok();
                pwm.enable();
            }

            // Delay for a second
            delay.delay_ms(250_u16);

            // Clear event triggering the interrupt
            exti.pr.write(|w| w.pr0().set_bit());
        }
    });
}

// Define an exception handler, i.e. function to call when exception occurs. Here, if our SysTick
// timer generates an exception the following handler will be called
#[exception]
fn SysTick() {
    // Enter critical section
    cortex_m::interrupt::free(|cs| {
        // Borrow access to our GPIO pin from the shared structure
        if let (Some(ref mut systick_out), Some(ref mut usart2), &mut Some(ref mut systick_count)) = (
            SYSTICK_OUT.borrow(cs).borrow_mut().deref_mut(),
            USART2.borrow(cs).borrow_mut().deref_mut(),
            SYSTICK_COUNT.borrow(cs).borrow_mut().deref_mut(),
        ) {
            systick_out.toggle().ok();
            // increment count
            *systick_count += 1;

            // Send pretty formatted message
            writeln!(usart2, "SysTick interrupt no. {}", *systick_count).unwrap()
        }
    });
}