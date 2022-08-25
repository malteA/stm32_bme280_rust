#![no_std]
#![no_main]
// #![no_unsafe]

use core::convert::Infallible;
use micromath::F32Ext;

use hal::{
    gpio::{Alternate, OpenDrain, Pin},
    pac::I2C1,
    timer::SysDelay,
};
use onewire::{ds18b20::split_temp, DeviceSearch, OneWire, DS18B20};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

use crate::hal::{
    i2c::{I2c, I2c1, I2cExt},
    pac,
    prelude::*,
    spi::*,
};

use cortex_m_rt::entry;

use arrform::{arrform, ArrForm};
use bme280_multibus::{Bme280, CHIP_ID};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
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

fn read_external_temperature(
    temp_sensor: &mut Option<DS18B20>,
    onewire: &mut OneWire<Infallible>,
    delay: &mut SysDelay,
) -> f32 {
    match temp_sensor {
        Some(sensor) => {
            let resolution = sensor.measure_temperature(onewire, delay).unwrap();
            delay.delay_ms(resolution.time_ms());
            let measurement = sensor.read_temperature(onewire, delay).unwrap();
            let (temperature, _fraction) = split_temp(measurement);
            return temperature.into();
        }
        None => return 0f32,
    }
}

fn calculate_dew_point(temperature: f32, humidity: f32) -> f32 {
    let c = 237.3f32;
    let b = 17.27f32;
    let dew_point = (c * ((humidity / 100f32).ln() + ((b * temperature) / (c + temperature))))
        / (b - ((humidity / 100f32).ln() + ((b * temperature) / (c + temperature))));
    return dew_point;
}

fn print_to_screen(
    display: &mut GraphicsMode<
        I2cInterface<
            I2c<
                I2C1,
                (
                    Pin<'B', 8, Alternate<4, OpenDrain>>,
                    Pin<'B', 9, Alternate<4, OpenDrain>>,
                ),
            >,
        >,
    >,
    position: Point,
    text: &str,
    text_style: MonoTextStyle<BinaryColor>,
) -> () {
    Text::with_baseline(text, position, text_style, Baseline::Top)
        .draw(display)
        .unwrap();
}

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

    bme.settings(&SETTINGS)
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

    print_to_screen(
        &mut display,
        Point::zero(),
        "Dew Point Controller",
        text_style,
    );
    print_to_screen(&mut display, Point::new(0, 40), "powered by", text_style);
    print_to_screen(
        &mut display,
        Point::new(0, 50),
        "stm32 and rust",
        text_style,
    );
    display.flush().unwrap();

    delay.delay_ms(500u32);

    // DS18B20
    let mut one_wire_pin = gpiob.pb7.into_open_drain_output();
    let mut onewire = OneWire::new(&mut one_wire_pin, false);

    if onewire.reset(&mut delay).is_err() {
        loop {
            rprintln!("ERROR");
        }
    }

    let mut search = DeviceSearch::new();

    let mut ds18b20 = if let Some(device) = onewire.search_next(&mut search, &mut delay).unwrap() {
        match device.address[0] {
            ds18b20::FAMILY_CODE => unsafe { Some(DS18B20::new_forced(device)) },
            _ => None,
        }
    } else {
        None
    };

    loop {
        let internal_measurement = bme.sample().expect("Failed to sample BME280");

        let external_temperature =
            read_external_temperature(&mut ds18b20, &mut onewire, &mut delay);
        rprintln!("external_temperature = {:#?}", external_temperature);

        let dew_point = calculate_dew_point(
            internal_measurement.temperature,
            internal_measurement.humidity,
        );
        rprintln!("dew_point = {:#?}", dew_point);

        display.clear();
        print_to_screen(
            &mut display,
            Point::zero(),
            arrform!(64, "temperature: {:.2}", internal_measurement.temperature).as_str(),
            text_style,
        );
        print_to_screen(
            &mut display,
            Point::new(0, 10),
            arrform!(64, "humidity: {:.2}", internal_measurement.humidity).as_str(),
            text_style,
        );
        print_to_screen(
            &mut display,
            Point::new(0, 20),
            arrform!(64, "dew point: {:.2}", dew_point).as_str(),
            text_style,
        );
        print_to_screen(
            &mut display,
            Point::new(0, 30),
            arrform!(64, "temperature o: {:.2}", external_temperature).as_str(),
            text_style,
        );
        let fan_status = match external_temperature < dew_point {
          true => "ON",
          false => "OFF"
        };
        print_to_screen(
            &mut display,
            Point::new(0, 50),
            arrform!(64, "FAN: {}", fan_status).as_str(),
            text_style,
        );
        display.flush().unwrap();
        rprintln!("sample = {:#?}", internal_measurement);
        delay.delay_ms(500u32);
    }
}
