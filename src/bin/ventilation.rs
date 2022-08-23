#![no_std]
#![no_main]
// #![no_unsafe]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

use crate::hal::{i2c::I2c, pac, prelude::*, spi::*};

use cortex_m_rt::entry;

use arrform::{arrform, ArrForm};
use bme280_multibus::{Bme280, CHIP_ID};
use embedded_graphics::{
  mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
  pixelcolor::BinaryColor,
  prelude::*,
  text::{Baseline, Text},
};
use one_wire_bus::OneWire;
use sh1106::{prelude::*, Builder};

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

  let pa5 = _gpioa.pa5.into_alternate(); // SCK
  let pa6 = _gpioa.pa6.into_alternate(); // MISO
  let pa7 = _gpioa.pa7.into_alternate().internal_pull_up(true); // MOSI
  let mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
  };

  let spi = Spi::new(dp.SPI1, (pa5, pa6, pa7), mode, 3.MHz(), &clocks);
  let mut bme: Bme280<_> = Bme280::from_spi(spi, _cs_0).expect("Failed to initialize BME280");
  bme.reset().expect("Failed to reset");
  rprintln!("after bme280_1");

  delay.delay_ms(2u32);

  // sanity check
  assert_eq!(bme.chip_id().expect("Failed to read chip ID"), CHIP_ID);
  rprintln!("Chip ID ok");

  bme
    .settings(&SETTINGS)
    .expect("Failed to initialize BME280");

  delay.delay_ms(250u32);

  // SSD1306 OLED
  let scl = gpiob.pb8.into_alternate_open_drain();
  let sda = gpiob.pb9.into_alternate_open_drain();

  let i2c = dp.I2C1.i2c((scl, sda), 4.kHz(), &clocks);

  let mut display: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();

  display.init().unwrap();
  display.flush().unwrap();

  let text_style = MonoTextStyleBuilder::new()
    .font(&FONT_6X10)
    .text_color(BinaryColor::On)
    .build();

  Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
    .draw(&mut display)
    .unwrap();

  Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
    .draw(&mut display)
    .unwrap();

  display.flush().unwrap();

  loop {
    let sample: bme280_multibus::Sample = bme.sample().expect("Failed to sample BME280");

    display.clear();
    Text::with_baseline(
      arrform!(64, "temperature: {:.2}", sample.temperature).as_str(),
      Point::zero(),
      text_style,
      Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();
    Text::with_baseline(
      arrform!(64, "humidity: {:.2}", sample.humidity).as_str(),
      Point::new(0, 16),
      text_style,
      Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();
    display.flush().unwrap();
    rprintln!("sample = {:#?}", sample);
    delay.delay_ms(500u32);
  }
}
