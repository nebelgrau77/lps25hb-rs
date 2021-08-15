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

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32l4xx_hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);


    let mut led = gpiob.pb3.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

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

    let mut delay = Delay::new(cp.SYST, clocks);
    
    let mut scl = gpioa.pa9.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    
    scl.internal_pull_up(&mut gpioa.pupdr, true);
    let scl = scl.into_af4(&mut gpioa.moder, &mut gpioa.afrh);

    let mut sda = gpioa.pa10.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    sda.internal_pull_up(&mut gpioa.pupdr, true);
    let sda = sda.into_af4(&mut gpioa.moder, &mut gpioa.afrh);

    let mut i2c = I2c::i2c1(dp.I2C1, (scl, sda), 100.khz(), clocks, &mut rcc.apb1r1);
    
    let i2c_interface = I2cInterface::init(i2c, I2cAddress::SA0_VCC); // Pololu board

    //let mut lps25hb = LPS25HBInit {..Default::default()}.with_interface(i2c_interface);
    //let mut lps25hb = LPS25HBInit {}.with_interface(i2c_interface);
    let mut lps25hb = LPS25HB::new(i2c_interface);

    lps25hb.software_reset().unwrap();

    delay.delay_ms(5 as u32);
    lps25hb.sensor_power(Control::On).unwrap();
    
    lps25hb.bdu_config(Control::On).unwrap();

    loop {
    
        /*

        lps25hb.enable_one_shot().unwrap();    

        let temp = lps25hb.read_temperature().unwrap();            
        let press = lps25hb.read_pressure().unwrap();

        */

        lps25hb.set_datarate(ODR::_7Hz).unwrap();
        //lps25hb.fifo_mode_config(FIFO_MODE::Bypass_to_FIFO).unwrap();

        let temp = lps25hb.read_temperature().unwrap();            
        let press = lps25hb.read_pressure().unwrap();

        //let my_mask = lps25hb.get_mask(ODR::_12_5Hz).unwrap();

        //let temp = lps25hb.read_temp().unwrap();            

        //let temp_l = lps25hb.read_register(Registers::TEMP_OUT_L).unwrap();
        //let temp_h = lps25hb.read_register(Registers::TEMP_OUT_H).unwrap();

        //let fifocontrol = lps25hb.read_register(Registers::FIFO_CTRL).unwrap();
        //let ctrlreg1 = lps25hb.read_register(Registers::CTRL_REG1).unwrap();

        led.set_high().ok();    
        
        delay.delay_ms(50 as u32);

        let whoami = lps25hb.get_device_id().unwrap();

        writeln!(tx, "my lucky number is {}\r", whoami).unwrap();

        //writeln!(tx, "temperature: {}\r", temp).unwrap();
        //writeln!(tx, "mask: {}\r", my_mask).unwrap();

        //writeln!(tx, "temperature: {:.2}, pressure: {:.2}\r", temp, press).unwrap();
        //writeln!(tx, "ctrl_reg1: {}, fifo_ctrl: {}\r", ctrlreg1, fifocontrol).unwrap();

        // writeln!(tx, "TEMP_L: {}, TEMP_H: {}\r", temp_l, temp_h).unwrap();

        led.set_low().ok();
        delay.delay_ms(50 as u32);

        

        }
}


