#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    timer::systimer::SystemTimer,
    analog::adc::{Adc, AdcConfig, Attenuation},
};
use log::info;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Setup onboard LED connected to GPIO8
    let mut led = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());

    // Setup ADC1 on GPIO2
    let analog_pin = peripherals.GPIO2;
    let mut adc1_config = AdcConfig::<esp_hal::peripherals::ADC1>::new();
    let mut pin = adc1_config.enable_pin(
        analog_pin,
        Attenuation::_11dB,
    );
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Main..");
    let _ = spawner;

    loop {
        match nb::block!(adc1.read_oneshot(&mut pin)) {
            Ok(pin_value) => info!("ADC Value: {}", pin_value),
            Err(e) => info!("ADC read error: {:?}", e),
        }
        led.toggle();
        Timer::after(Duration::from_millis(200)).await;
    }
}
