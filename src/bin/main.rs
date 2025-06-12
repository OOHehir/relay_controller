#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    timer::systimer::SystemTimer,
};
use log::info;
use static_cell::StaticCell;

const ON_THRESHOLD: u16 = 2400;
const OFF_THRESHOLD: u16 = 1800;

#[embassy_executor::task]
async fn led_func(mut led: Output<'static>) {
    info!("LED task started");
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn control_relay(
    mut relay: Output<'static>,
    control: &'static Signal<CriticalSectionRawMutex, bool>,
) {
    control.wait().await;
    info!("Relay control signal received, activating relay...");
    loop {
        // Wait for the signal to be set
        control.wait().await;
        info!("Relay activated");
        relay.set_high();
        Timer::after(Duration::from_secs(5)).await;
        relay.set_low();
        info!("Relay deactivated");
        // Reset the control signal
        control.reset();
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Setup onboard LED connected to GPIO8
    let led = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());

    // Setup relay GPIO connected to GPIO1
    let relay = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());

    // Setup ADC1 on GPIO0
    let analog_pin = peripherals.GPIO0;
    let mut adc1_config = AdcConfig::<esp_hal::peripherals::ADC1>::new();
    let mut pin = adc1_config.enable_pin(analog_pin, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Main..");
    let _ = spawner;
    spawner.spawn(led_func(led)).ok();

    static RELAY_CTRL: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
    let relay_ctrl_signal = &*RELAY_CTRL.init(Signal::new());
    spawner.spawn(control_relay(relay, relay_ctrl_signal)).ok();

    let mut previous_value: u16 = 0;
    let mut was_running: bool = false;
    loop {
        match nb::block!(adc1.read_oneshot(&mut pin)) {
            Ok(pin_value) => {
                if pin_value != previous_value {
                    previous_value = pin_value;
                    info!("ADC Value: {}", pin_value);
                    if pin_value > ON_THRESHOLD {
                        was_running = true;
                    }
                    if pin_value < OFF_THRESHOLD && was_running {
                        // i.e. just switched off, trigger once only
                        info!("Less than {} & previous was running", OFF_THRESHOLD);
                        // Activate relay
                        relay_ctrl_signal.signal(true);
                        was_running = false; // Reset the running state
                    }
                }
            }
            Err(e) => info!("ADC read error: {:?}", e),
        }

        // Need a delay or otherwise spawned tasks will starve
        Timer::after(Duration::from_millis(200)).await;
    }
}
