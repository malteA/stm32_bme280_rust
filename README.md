# Getting started

## install openocd

```sh
pacman -S openocd
```

## install probe-run

```sh
cargo install probe-run
```

## install target

```sh
rustup target add thumbv7em-none-eabihf
```

## run

```sh
cargo run
```

## wiring

### blackpill w/ bme280

device|VCC|GND|SCK|MOSI|CS|MISO
|---|---|---|---|---|---|---|
blackpill|3v3|GND|PA5|PA7|PB0|PA6
bme280|VCC|GND|SCL|SDA|CSB|SDO

### blackpill w/ stlink

device|3v3|SWCLK|GND|SWDIO
|---|---|---|---|---|
blackpill|3v3|SCK|GND|DIO
ST-Link|3v3|SWCLK|GND|SWDIO

### blackpill w/ sh1106 oled

device|3v3|GND|SCL|SDA
|---|---|---|---|---|
blackpill|3v3|GND|PB8|PB9
SH1106|VDO|GND|SCK|SDA
