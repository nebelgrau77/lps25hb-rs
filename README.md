# Rust LPS25HB pressure sensor driver

![Maintenance Intention](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg)

A platform agnostic Rust driver for the ST Microelectronics LPS25HB pressure sensor,
based on the [`embedded-hal`] traits.

[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal

Inspired by and partially based on [another STMicroelectronics MEMS driver](https://github.com/lonesometraveler/lsm9ds1).

This driver allows you to:
- ~~read pressure and temperature~~
- ~~check if sensor is reachable~~

# EXPERIMENTAL - MAY NOT WORK!!!

At the moment it is just copied from the existing and working LPS22HB, may need modifications.

## WORK IN PROGRESS:

This library is work in progress. Not all features are implemented yet. Contributions are welcome.

### TO DO:
- [ ] output data rate setting
- [ ] interrupts configuration
- [ ] FIFO configuration
- [ ] reading reference pressure
- [ ] calibration
- [ ] reading data ready status
- [ ] reading data overrun status

[Some blog post](https://nebelgrau77.github.io/posts/rust_driver/)

## The device

The LPS25HB is an ultra-compact piezoresistive absolute pressure sensor which functions as a digital output barometer. The device comprises a sensing element and an IC interface which communicates through I2C or SPI from the sensing element to the application.

Datasheet: [LPS25HB](https://www.st.com/resource/en/datasheet/lps25hb.pdf)

## Usage

To use this driver, import this crate and an `embedded_hal` implementation,
then instantiate the device.

Please find additional examples using hardware in this repository: [examples]

[examples]: https://github.com/nebelgrau77/lps22hb-rs/tree/main/examples

## Support

For questions, issues, feature requests, and other changes, please file an
[issue in the github project](https://github.com/nebelgrau77/lps22hb-rs/issues).

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT) at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
