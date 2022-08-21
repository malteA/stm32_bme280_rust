#![no_std]
#![no_main]
// #![no_unsafe]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

use crate::hal::{pac, prelude::*, spi::*};

use cortex_m_rt::entry;

use bme280_multibus::{Bme280, CHIP_ID};

const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
  config: bme280_multibus::Config::reset()
    .set_standby_time(bme280_multibus::Standby::Millis125)
    .set_filter(bme280_multibus::Filter::X8),
  ctrl_meas: bme280_multibus::CtrlMeas::reset()
    .set_osrs_t(bme280_multibus::Oversampling::X8)
    .set_osrs_p(bme280_multibus::Oversampling::X8)
    .set_mode(bme280_multibus::Mode::Normal),
  ctrl_hum: bme280_multibus::Oversampling::X8,
};

#[entry]
fn main() -> ! {
  rtt_init_print!();
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.freeze();
  let mut delay = cp.SYST.delay(&clocks);

  let _gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  let _led = gpioc.pc13.into_push_pull_output();
  let _cs_0 = gpiob.pb0.into_push_pull_output();
  let _cs_1 = gpiob.pb1.into_push_pull_output();
  let _cs_2 = gpiob.pb2.into_push_pull_output();

  let pa5 = _gpioa.pa5.into_push_pull_output(); // SCK
  let pa6 = _gpioa.pa6.into_alternate(); // MISO
  let pa7 = _gpioa.pa7.into_alternate(); // MOSI
  let mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
  };

  let spi1 = Spi::new(dp.SPI1, (pa5, pa6, pa7), mode, 10.Hz(), &clocks);

  let mut bme: Bme280<_> = Bme280::from_spi(spi1, _cs_0).expect("Failed to initialize BME280");
  bme.reset().expect("Failed to reset");
  rprintln!("after bme280");

  delay.delay_ms(2u32);

  // sanity check
  assert_eq!(bme.chip_id().expect("Failed to read chip ID"), CHIP_ID);
  rprintln!("Chip ID ok");

  bme
    .settings(&SETTINGS)
    .expect("Failed to initialize BME280");

  delay.delay_ms(250u32);

  loop {
    let sample: bme280_multibus::Sample = bme.sample().expect("Failed to sample BME280");
    rprintln!("sample = {:#?}", sample);
    delay.delay_ms(500u32);
  }
}
