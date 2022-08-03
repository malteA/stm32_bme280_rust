#![no_std]
#![no_main]
// #![no_unsafe]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

use crate::hal::{
  gpio::{
    self,
    gpiob::{PB6, PB7},
    AF1,
  },
  i2c::I2c,
  pac,
  pac::{EXTI, I2C1},
  prelude::*,
  *,
};

use cortex_m_rt::entry;

// #[entry]
// fn main() -> ! {
//     rtt_init_print!();

//     rprintln!("Program started!");

//     let dp = pac::Peripherals::take().unwrap();
//     let cp = cortex_m::Peripherals::take().unwrap();

//     let rcc = dp.RCC.constrain();

//     let clocks = rcc.cfgr.freeze();

//     let gpioc = dp.GPIOC.split();
//     let mut led = gpioc.pc13.into_push_pull_output();
//     let mut delay = cp.SYST.delay(&clocks);

//     loop {
//         rprintln!("Blink loop...");
//         for _ in 1..6 {
//           delay.delay_ms(200u32);
//           led.toggle();
//         }
//         for _ in 1..6 {
//           delay.delay_ms(1000u32);
//           led.toggle();
//         }
//     }
// }

use bme280_multibus::{Bme280, Sample, CHIP_ID};
use embedded_hal::digital::v2::OutputPin;
// use ftdi_embedded_hal::{FtHal, OutputPin, Spi};
// use libftd2xx::Ft232h;

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

struct Local {
  exti: EXTI,
  bme280: Bme280<bme280_multibus::i2c::Bme280Bus<I2c<I2C1, (PB6<AF1>, PB7<AF1>)>>>,
}

#[entry]
fn main() -> ! {
  rtt_init_print!();
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  let rcc = dp.RCC.constrain();

  // let ccdr = rcc.cfgr.sysclk(100.MHz()).freeze();
  let mut clocks = rcc.cfgr.freeze();

  // The 61440 kHz frequency can be divided to get exactly 48 kHz sample rate even when
  // generating master clock
  // let clock = rcc
  //     .cfgr
  //     .use_hse(8u32.MHz())
  //     .sysclk(96.MHz())
  //     .i2s_clk(61440.kHz())
  //     .freeze();
  let mut delay = cp.SYST.delay(&clocks);

  // let device: Ft232h = libftd2xx::Ftdi::new().unwrap().try_into().unwrap();
  // let hal_dev: FtHal<Ft232h> = FtHal::init_default(device).unwrap();

  let gpioa = dp.GPIOA.split();
  let gpiob = dp.GPIOB.split();
  let gpioc = dp.GPIOC.split();

  // let cs = gpioc.pc13.
  let mut led = gpioc.pc13.into_push_pull_output();
  let mut cs = gpiob.pb0.into_push_pull_output();

  // let spi1_pins = (
  //     gpioa.pa5.into_alternate(), // W5500 SCK
  //     gpioa.pa6.into_alternate(), // W5500 MISO
  //     gpioa.pa7.into_alternate(), // W5500 MOSI
  // );
  let i2c1_pins = (
    gpiob.pb6, // I2C SCL
    gpiob.pb7, // I2C SDA
  );

  // let spi1 = Spi::spi1(dp.SPI1, spi1_pins, W5500_MODE, 1.mhz(), &mut rcc);
  // let foo = hal::i2c::I2c::
  let i2c = dp.I2C1.i2c(i2c1_pins, 100.kHz(), &mut clocks);
  // let spi: Spi<Ft232h> = hal_dev.spi().unwrap();
  // let cs: OutputPin<Ft232h> = hal_dev.ad3().unwrap();

  let mut bme: Bme280<_> = Bme280::from_i2c(i2c, bme280_multibus::i2c::Address::SdoGnd)
    .expect("Failed to initialize BME280");
  bme.reset().expect("Failed to reset");

  delay.delay_ms(2u32);
  // std::thread::sleep(std::time::Duration::from_millis(2));

  // sanity check
  assert_eq!(bme.chip_id().expect("Failed to read chip ID"), CHIP_ID);
  rprintln!("Chip ID ok");

  bme
    .settings(&SETTINGS)
    .expect("Failed to initialize BME280");

  delay.delay_ms(250u32);
  // std::thread::sleep(std::time::Duration::from_millis(250));

  let sample: bme280_multibus::Sample = bme.sample().expect("Failed to sample BME280");
  rprintln!("sample = {:#?}", sample);

  loop {
    rprintln!("Blink loop...");
    delay.delay_ms(500u32);
  }
}
