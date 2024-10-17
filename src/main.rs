#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, Config, Ipv4Address, Stack, StackResources};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Input, Io, Level, Output, Pull},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::{
    wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
        WifiState,
    },
    EspWifiInitFor,
};

use static_cell::StaticCell;

const SSID: &'static str = env!("SSID");
const PASSWORD: &'static str = env!("PASSWORD");

const CMD_POWER_ON : &[u8] = b"GET /YamahaExtendedControl/v1/main/setPower?power=on HTTP/1.1\r\nHost: 192.168.50.201\r\n\r\n";

/// Measures the distance using the ultrasonic sensor and controls the RGB LED based on the measured distance.
///
/// This function uses GPIO pins 4 and 5 for the ultrasonic sensor and pins 6, 7, and 8 for the RGB LED.
/// It sends a pulse to the trigger pin (GPIO 4) and measures the time it takes for the echo pin (GPIO 5)
/// to go high and then fall back down. The time difference is used to calculate the distance in millimeters.
///
/// The RGB LED is controlled based on the distance:
/// * Green: Distance less than 125 mm
/// * Blue: Distance between 125 mm and 205 mm
/// * Red: Distance greater than 205 mm
///
/// The distance measurement is repeated every second.
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
        Delay.delay_us(1).await;
        trigger.set_low();

        echo.wait_for_high().await; // TODO: timeout?
        t1 = Instant::now();
        echo.wait_for_falling_edge().await; // TODO: timeout?
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
        Timer::after(Duration::from_millis(1000)).await;
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

    // Init and connect to WIFI
    // relevant Wifi example:
    //   https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/wifi_embassy_bench.rs
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
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&wifi_init, wifi, WifiStaDevice).unwrap();

    let config = Config::dhcpv4(Default::default());
    let seed = 1234; // very random, very secure seed

    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    static STACK: StaticCell<Stack<WifiDevice<'_, WifiStaDevice>>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        wifi_interface,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    ));

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(&stack)).ok();

    dhcp_handshake(&stack).await;

    let mut http_response = [0u8; 1024];
    match send_http_command(&stack, CMD_POWER_ON, &mut http_response).await {
        Ok(bytes) => println!("{}", core::str::from_utf8(&http_response[..bytes]).unwrap()),
        Err(e) => println!("HttpCmdError: {:?}", e),
    }

    spawner.spawn(measure_distance(io)).ok();

    loop {
        Timer::after(Duration::from_millis(5000)).await;
    }
}

#[derive(Debug)]
enum HttpCmdError {
    Connect(embassy_net::tcp::ConnectError),
    Tcp(embassy_net::tcp::Error),
    HttpError(usize),
}

impl From<embassy_net::tcp::ConnectError> for HttpCmdError {
    fn from(e: embassy_net::tcp::ConnectError) -> Self {
        HttpCmdError::Connect(e)
    }
}

impl From<embassy_net::tcp::Error> for HttpCmdError {
    fn from(e: embassy_net::tcp::Error) -> Self {
        HttpCmdError::Tcp(e)
    }
}

/// Starts a new TCP socket and connects to the specified remote endpoint.
///
///  This function works on a best effort base. If an error occurs, an error message is printed to the
///  console.
///
/// # Arguments
///
/// * `stack`: The network stack
/// * `http_cmd`: The HTTP command to send
/// * `http_response`: The buffer where the response will be stored
async fn send_http_command(
    stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>,
    http_cmd: &[u8],
    http_response: &mut [u8],
) -> Result<usize, HttpCmdError> {
    let remote_endpoint = (Ipv4Address::new(192, 168, 50, 201), 80);
    let mut rx_buffer = [0u8; 4096];
    let mut tx_buffer = [0u8; 4096];
    let mut socket = TcpSocket::new(&stack, &mut rx_buffer, &mut tx_buffer);
    socket.set_timeout(Some(embassy_time::Duration::from_secs(2)));

    socket.connect(remote_endpoint).await?;
    socket.write_all(http_cmd).await?;
    let bytes = socket.read(http_response).await?;

    // http_response starts with "HTTP/1.1 200 OK\r\n"
    let mut ret_code: usize = 0;
    if bytes >= 12 {
        if let Ok(r_str) = core::str::from_utf8(&http_response[9..=11]) {
            if let Ok(r) = str::parse::<usize>(r_str) {
                ret_code = r;
                if ret_code == 200 {
                    return Ok(bytes);
                }
            }
        }
    }
    // if we reach this point something is wrong with the return code
    println!(
        "Non sucessful return code: {}\n{}",
        ret_code,
        core::str::from_utf8(&http_response[..bytes]).unwrap()
    );
    Err(HttpCmdError::HttpError(ret_code))
}

/// Connects (and maintains connection) to the WiFi network.
///
/// This function will continuously attempt to connect to the WiFi network defined by the `SSID`
/// and `PASSWORD` environment variables. On a disconnection event if will try to reconnect.
///
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait here for an eventual disconnect and connect again afterwards
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => (),
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

/// Runs the network stack.
///
/// This function starts the network stack and runs it continuously.
/// This is required to process network events.
///
/// # Arguments
///
/// * `stack`: The network stack
#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}

/// Performs a DHCP request to obtain an IP address.
///
/// # Arguments
///
/// * `stack`: The network stack
async fn dhcp_handshake(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(100)).await;
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
