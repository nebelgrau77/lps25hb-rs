/* talking to the LPS25HB module over I2C
*/


#![no_main]
#![no_std]

use cortex_m;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32l4xx_hal::{
    delay::Delay,
    prelude::*,
    serial::{Config, Serial},
    i2c::I2c,
    };

use core::fmt::Write;

use lps25hb::*;

use lps25hb::interface::{I2cInterface,
    i2c::I2cAddress};

use lps25hb::sensor::*;
use lps25hb::register::*;
use lps25hb::interrupt::*;
use lps25hb::fifo::*;

#[entry]
fn main() -> ! {
    // set up the board peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32l4xx_hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);

    // configure built-in LED 
    let mut led = gpiob.pb3.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    // configure USART transmission
    let tx = gpioa.pa2.into_af7(&mut gpioa.moder, &mut gpioa.afrl);
    let rx = gpioa.pa3.into_af7(&mut gpioa.moder, &mut gpioa.afrl);

    let serial = Serial::usart2(
        dp.USART2,
        (tx,rx),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb1r1,
    );

    let (mut tx, mut rx) = serial.split();

    // delay provider
    let mut delay = Delay::new(cp.SYST, clocks);
    
    // configure I2C bus
    let mut scl = gpioa.pa9.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    scl.internal_pull_up(&mut gpioa.pupdr, true);
    let scl = scl.into_af4(&mut gpioa.moder, &mut gpioa.afrh);

    let mut sda = gpioa.pa10.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    sda.internal_pull_up(&mut gpioa.pupdr, true);
    let sda = sda.into_af4(&mut gpioa.moder, &mut gpioa.afrh);

    let mut i2c = I2c::i2c1(dp.I2C1, (scl, sda), 100.khz(), clocks, &mut rcc.apb1r1);

    
    // configure I2C interface for the LPS25HB driver
    let i2c_interface = I2cInterface::init(i2c, I2cAddress::SA0_VCC); // Pololu board

    // create a new driver instance with the I2C interface    
    let mut lps25hb = LPS25HB::new(i2c_interface);

    // turn the sensor on 
    lps25hb.sensor_on(true).unwrap();
    
    // enable Block Data Update
    lps25hb.bdu_enable(true).unwrap(); 

    // set data rate to 1Hz
    lps25hb.set_datarate(ODR::_1Hz).unwrap();


    // configure the interrupt pin with default settings, enable data ready signal on the pin

    let int_conf = InterruptConfig{enable_data_ready: true,
                                    ..Default::default()};
    
    lps25hb.enable_interrupts(true, int_conf).unwrap();

    // configure the FIFO in Mean mode, with 8 samples taken for the mean

    let fifo_conf = FIFOConfig{fifo_mode: FIFO_MODE::FIFO_Mean,
                                fifo_mean_config: FIFO_MEAN::_8sample,
                                ..Default::default()};

    lps25hb.enable_fifo(true, fifo_conf).unwrap();

    // THIS SHOULD GENERATE AN INTERRUPT AFTER 8 SECONDS (8 SAMPLES FOR THE MEAN, DATA READY INTERRUPT)

    loop {
            
        // read temperature and pressure
        let temp = lps25hb.read_temperature().unwrap();            
        let press = lps25hb.read_pressure().unwrap();

        let id = lps25hb.get_device_id().unwrap();

        // LED on
        led.set_high().ok();    
        
        // print data to serial
        writeln!(tx, "temperature: {:.1} C, pressure: {:.1} hPa\r", temp, press).unwrap();
        //writeln!(tx, "my name is: {}\r", id).unwrap();
        
        // wait a little
        delay.delay_ms(100 as u32);

        // LED off, wait some more
        led.set_low().ok();
        delay.delay_ms(50 as u32);


        }
}
