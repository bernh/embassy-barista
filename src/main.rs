#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

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
        utils::create_network_interface, AccessPointInfo, ClientConfiguration, Configuration,
        WifiError, WifiStaDevice,
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
        println!("distance: {distance} mm");
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
    let timer1: ErasedTimer = timg1.timer0.into();
    // only using a single timer works as well, but I assume there is some reason
    // that multiple ones can be provided to embassy.
    let timers = [timer0];
    let timers = make_static!(timers);
    esp_hal_embassy::init(&clocks, timers);

    // relevant Wifi examples:
    // - https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/wifi_static_ip.rs
    let wifi_init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        PeriodicTimer::new(timer1),
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&wifi_init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
    let mut wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    });
    let res = controller.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());

    println!("Start Wifi Scan");
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }

    println!("{:?}", controller.get_capabilities());
    println!("wifi_connect {:?}", controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());

    println!("Setting static IP {}", STATIC_IP);

    wifi_stack
        .set_iface_configuration(&esp_wifi::wifi::ipv4::Configuration::Client(
            esp_wifi::wifi::ipv4::ClientConfiguration::Fixed(
                esp_wifi::wifi::ipv4::ClientSettings {
                    ip: esp_wifi::wifi::ipv4::Ipv4Addr::from(parse_ip(STATIC_IP)),
                    subnet: esp_wifi::wifi::ipv4::Subnet {
                        gateway: esp_wifi::wifi::ipv4::Ipv4Addr::from(parse_ip(GATEWAY_IP)),
                        mask: esp_wifi::wifi::ipv4::Mask(24),
                    },
                    dns: None,
                    secondary_dns: None,
                },
            ),
        ))
        .unwrap();

    println!(
        "Start busy loop on main. Point your browser to http://{}:8080/",
        STATIC_IP
    );

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    socket.listen(8080).unwrap();

    spawner.spawn(measure_distance(io)).ok();

    loop {
        socket.work();

        if !socket.is_open() {
            socket.listen(8080).unwrap();
        }

        if socket.is_connected() {
            println!("Connected");

            let mut time_out = false;
            let wait_end = current_millis() + 20 * 1000;
            let mut buffer = [0u8; 1024];
            let mut pos = 0;
            loop {
                if let Ok(len) = socket.read(&mut buffer[pos..]) {
                    let to_print =
                        unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

                    if to_print.contains("\r\n\r\n") {
                        println!("{}", to_print);
                        break;
                    }

                    pos += len;
                } else {
                    break;
                }

                if current_millis() > wait_end {
                    println!("Timeout");
                    time_out = true;
                    break;
                }
            }

            if !time_out {
                socket.write_all(
                    b"HTTP/1.0 200 OK\r\n\r\n\
                    <html>\
                        <body>\
                            <h1>Hello Rust! Hello esp-wifi!</h1>\
                            <img src=\"https://rustacean.net/more-crabby-things/dancing-ferris.gif\"/>
                        </body>\
                    </html>\r\n\
                    "
                ).unwrap();

                socket.flush().unwrap();
            }

            socket.close();

            println!("Done\n");
            println!();
        }

        let wait_end = current_millis() + 5 * 1000;
        while current_millis() < wait_end {
            socket.work();
        }
    }

    // let mut count = 0;
    // loop {
    //     esp_println::println!("Main Task Count: {}", count);
    //     count += 1;
    //     Timer::after(Duration::from_millis(5000)).await;
    // }
}

fn parse_ip(ip: &str) -> [u8; 4] {
    let mut result = [0u8; 4];
    for (idx, octet) in ip.split(".").into_iter().enumerate() {
        result[idx] = u8::from_str_radix(octet, 10).unwrap();
    }
    result
}
