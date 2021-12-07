# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- interrupt configuration struct
- enable_interrupts function that enables and configures the interrupt pin 
- FIFO configuration struct
- enable_fifo function that enables and configures the FIFO 

## [0.0.4] - 2021-12-04

### Added
- reading threshold for pressure interrupt generation
- setting threshold for pressure interrupt generation
- setting pressure offset value
- added Raspberry Pi example

[0.0.4]: https://github.com/nebelgrau77/lps25hb-rs/releases/tag/v0.0.4

## [0.0.3] - 2021-08-16

### Added
- Reading FIFO stored data level
- Reading pressure offset value

### Changed
- Bitmasks are inside a struct
- All "config/control" functions become "_enable", with a boolean parameter. 
- Moved enums into lib.rs 

## Removed
- `Control` enum

[0.0.3]: https://github.com/nebelgrau77/lps25hb-rs/releases/tag/v.0.0.3

## [0.0.2] - 2021-08-15

### Added
- Read pressure and temperature
- Power down control
- Data rate setting
- Temperature and pressure resolution setting
- Reboot and software reset functions
- Enable one-shot reading
- Enable/disable interrupts generation
- Interrupts configuration
- Signals on DRDY pin setting
- Autozero function config and reset
- SPI configuration (SPI interface not implemented yet)
- Data availability and events occurence status reading
- FIFO enabling/disabling
- FIFO configuration
- FIFO status reading
- FIFO interrupts settings


[0.0.2]: https://github.com/nebelgrau77/lps25hb-rs/releases/tag/v.0.0.2

## [0.0.1] - 2021-08-03

### Added
- Basic read and write functions
- Registers and bitmasks definitions
- Read device ID function
- ~~Checking if sensor is reachable (WHOAMI)~~
- ~~Enabling single-shot data acquisition~~
- ~~Reading pressure and temperature~~

