#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_imports)]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_async::delay::DelayNs;
use embedded_io::{Read, Write};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Input, Io, Level, Output, Pull},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer, PeriodicTimer},
};
use esp_println::println;
use esp_wifi::{
    current_millis,
    wifi::{
        get_wifi_state, utils::create_network_interface, AccessPointInfo, ClientConfiguration,
        Configuration, WifiController, WifiError, WifiEvent, WifiStaDevice, WifiState,
    },
    wifi_interface::WifiStack,
    EspWifiInitFor,
};
use smoltcp::iface::SocketStorage;
use static_cell::make_static;

const SSID: &'static str = env!("SSID");
const PASSWORD: &'static str = env!("PASSWORD");
const STATIC_IP: &'static str = env!("STATIC_IP");
const GATEWAY_IP: &'static str = env!("GATEWAY_IP");

#[embassy_executor::task]
async fn measure_distance(io: Io) {
    let mut trigger = Output::new(io.pins.gpio3, Level::Low);
    let mut echo = Input::new(io.pins.gpio5, Pull::Down);

    let mut r_pin = Output::new(io.pins.gpio6, Level::Low);
    let mut g_pin = Output::new(io.pins.gpio7, Level::Low);
    let mut b_pin = Output::new(io.pins.gpio8, Level::Low);

    let mut t1: Instant;
    let mut t2: Instant;

    loop {
        // println!("entering measurment loop");
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
            x if x < 125.0 => {
                r_pin.set_low();
                g_pin.set_high(); // GREEN (full: 45 mm)
                b_pin.set_low();
            }
            x if x < 205.0 => {
                r_pin.set_low();
                g_pin.set_low();
                b_pin.set_high(); // BLUE
            }
            _ => {
                r_pin.set_high(); // RED (empty: 240 mm)
                g_pin.set_low();
                b_pin.set_low();
            }
        }
        println!("distance: {distance} mm");
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Init embassy and kick-off periodic measurement task
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0.timer0);
    spawner.spawn(measure_distance(io)).ok();

    // Init and connect to WIFI
    // relevant Wifi examples:
    // - https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/wifi_static_ip.rs
    // - https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/wifi_embassy_bench.rs
    let timg1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let wifi_init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        timg1.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&wifi_init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();

    spawner.spawn(connection(controller)).ok();

    // let mut wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let mut count = 0;
    loop {
        esp_println::println!("Main Task Count: {}", count);
        count += 1;
        Timer::after(Duration::from_millis(5000)).await;
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.get_capabilities());

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    });
    controller.set_configuration(&client_config).unwrap();
    println!("Starting wifi");
    controller.start().unwrap();
    println!("Wifi started!");

    println!("About to connect...");
    match controller.connect() {
        Ok(_) => println!("Wifi connected!"),
        Err(e) => {
            println!("Failed to connect to wifi: {e:?}");
        }
    }
}

// fn parse_ip(ip: &str) -> [u8; 4] {
//     let mut result = [0u8; 4];
//     for (idx, octet) in ip.split(".").into_iter().enumerate() {
//         result[idx] = u8::from_str_radix(octet, 10).unwrap();
//     }
//     result
// }
