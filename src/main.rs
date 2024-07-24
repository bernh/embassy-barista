#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_async::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Input, Io, Level, Output, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, OneShotTimer},
};
use static_cell::make_static;

#[embassy_executor::task]
async fn measure_distance(io: Io) {
    let mut trigger = Output::new(io.pins.gpio4, Level::Low);
    let mut echo = Input::new(io.pins.gpio5, Pull::Down);

    let mut r_pin = Output::new(io.pins.gpio6, Level::Low);
    let mut g_pin = Output::new(io.pins.gpio7, Level::Low);
    let mut b_pin = Output::new(io.pins.gpio8, Level::Low);

    let mut t1: Instant;
    let mut t2: Instant;

    loop {
        trigger.set_high();
        Delay.delay_us(1).await; // why is this working? Delay is a struct type but no instance...
        trigger.set_low();

        echo.wait_for_high().await;
        t1 = Instant::now();
        echo.wait_for_falling_edge().await;
        t2 = Instant::now();

        let duration = t2.duration_since(t1);
        let distance = f32::from(duration.as_micros() as u16) * 0.343 / 2.0; // with t in [us] -> distance in [mm]
        match distance {
            x if x < 100.0 => {
                r_pin.set_low();
                g_pin.set_high(); // GREEN
                b_pin.set_low();
            }
            x if x < 200.0 => {
                r_pin.set_low();
                g_pin.set_low();
                b_pin.set_high(); // BLUE
            }
            _ => {
                r_pin.set_high(); // RED
                g_pin.set_low();
                b_pin.set_low();
            }
        }
        esp_println::println!("distance: {distance} mm");
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timg1 = TimerGroup::new(peripherals.TIMG1, &clocks, None);
    let timer1 = OneShotTimer::new(timg1.timer0.into());
    // only using a single timer works as well, but I assume there is some reason
    // that multiple ones can be provided to embassy.
    let timers = [timer0, timer1];
    let timers = make_static!(timers);
    esp_hal_embassy::init(&clocks, timers);

    spawner.spawn(measure_distance(io)).ok();

    let mut count = 0;
    loop {
        esp_println::println!("Main Task Count: {}", count);
        count += 1;
        Timer::after(Duration::from_millis(5000)).await;
    }
}
