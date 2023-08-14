// #![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]


use panic_rtt_target as _;
use rtt_target::rtt_init_print;
// use rtt_target::rprintln;
use stm32f1xx_hal:: {
        prelude::*,
        gpio::*,
        gpio::PinState::{High, Low},
        usb::*,
        // afio::*,
        // timer::*,
        adc::*,
        i2c::*,
    };
use rtic_monotonics::systick::*;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use pcf857x::{Pcf8574, SlaveAddr};
use mcp4725::*;
use shared_bus;

use rata::vfd_driver::{VfdDriver, VfdControl};


#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [CAN_RX1])]
mod app {
    use super::*;


    #[shared]
    struct Shared {
        disp: VfdControl
    }

    // Local resources go here
    #[local]
    struct Local {
        beep: Pin<'C', 13, Output>,
        switch: Pin<'B', 7, Input>,
        usb_bus: usb_device::class_prelude::UsbBusAllocator<UsbBus<Peripheral>>,
        high1: Pin<'A', 8, Output>,
        low1: Pin<'B', 13, Output>,
        high2: Pin<'B', 14, Output>,
        low2: Pin<'A', 9, Output>,
        heater_voltage: Pin<'B', 1, Analog>,
        adc: Adc<you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::ADC1>,
        dac: MCP4725<BlockingI2c<you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::I2C2, (Pin<'B', 10, Alternate<OpenDrain>>, Pin<'B', 11, Alternate<OpenDrain>>)>>,
        i2c_bus: shared_bus::BusManager<shared_bus::NullMutex<BlockingI2c<you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::I2C1, (Pin<'B', 8, Alternate<OpenDrain>>, Pin<'B', 9, Alternate<OpenDrain>>)>>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {

        rtt_init_print!();

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let mut afio = cx.device.AFIO.constrain();
 
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .freeze(&mut flash.acr);

        if !clocks.usbclk_valid() {
            panic!("Clock parameter values are wrong!");
        }

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, clocks.sysclk().to_Hz(), systick_token);

        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpioc = cx.device.GPIOC.split();
        
        let _heater_disable = gpiob.pb15.into_push_pull_output_with_state(&mut gpiob.crh, Low);
        let mut amp_enable = gpioa.pa1.into_open_drain_output_with_state(&mut gpioa.crl, Low);
        let beep = gpioc.pc13.into_push_pull_output_with_state(&mut gpioc.crh, Low);
        let switch = gpiob.pb7.into_floating_input(&mut gpiob.crl);
        let high1 = gpioa.pa8.into_push_pull_output_with_state(&mut gpioa.crh, High);
        let low1 = gpiob.pb13.into_push_pull_output_with_state(&mut gpiob.crh, High);
        let high2 = gpiob.pb14.into_push_pull_output_with_state(&mut gpiob.crh, High);
        let low2 = gpioa.pa9.into_push_pull_output_with_state(&mut gpioa.crh, High);
        let heater_voltage = gpiob.pb1.into_analog(&mut gpiob.crl);
        let i2c1_sck = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let i2c1_sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
        let i2c2_sck = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let i2c2_sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
        

        let mut delay = cx.device.TIM2.delay_us(&clocks);
        delay.delay_ms(500u32);
        let _usb_on = gpioa.pa10.into_push_pull_output_with_state(&mut gpioa.crh, Low);

 
        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
        };
        let usb_bus = UsbBus::new(usb);

        let adc = Adc::adc1(cx.device.ADC1, clocks);

        let i2c = BlockingI2c::i2c1(
            cx.device.I2C1,
            (i2c1_sck, i2c1_sda),
            &mut afio.mapr, 
            Mode::Fast {
                frequency: 400.kHz(),
                duty_cycle: DutyCycle::Ratio16to9,
            }, 
            clocks,
            1000,
            10,
            1000,
            1000);
        let i2c_bus = shared_bus::BusManagerSimple::new(i2c);
  
        let audio_i2c = BlockingI2c::i2c2(
            cx.device.I2C2,
            (i2c2_sck, i2c2_sda),
            Mode::Fast {
                frequency: 400.kHz(),
                duty_cycle: DutyCycle::Ratio16to9,
            }, 
            clocks,
            1000,
            10,
            1000,
            1000);
        let mut dac = MCP4725::new(audio_i2c, 0b000);
        dac.set_dac_fast(PowerDown::Normal, 0x000).ok();
        amp_enable.set_high();

        let disp = VfdControl::new();


        beep::spawn().ok();
        // usb::spawn().ok();
        heater::spawn().ok();
        audio::spawn().ok();
        disp::spawn().ok();
        vfd::spawn().ok();

        (
            Shared {
                disp
            },
            Local {
                beep,
                switch,
                usb_bus,
                high1,
                low1,
                high2,
                low2,
                heater_voltage,
                adc,
                dac,
                i2c_bus,
            },
        )

    }

    // #[task(local = [beep], priority = 1)]
    // async fn beep(cx: beep::Context) {

    //     let led = cx.local.beep;

    //     loop {
    //         led.toggle();

    //         Systick::delay(1000.millis()).await;
    //     }
        
    // }

    #[task(local = [beep], priority = 1)]
    async fn beep(cx: beep::Context) {

        let beep = cx.local.beep;

        loop {

            beep.set_high();
            Systick::delay(1.millis()).await;

            beep.set_low();
            Systick::delay(1000.millis()).await;
        }
    }

    #[task(local = [usb_bus], priority = 1)]
    async fn usb(cx: usb::Context) {

        let usb_bus = cx.local.usb_bus;

        let mut serial = SerialPort::new(&usb_bus);

        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        loop {

            Systick::delay(5.millis()).await;

            if !usb_dev.poll(&mut [&mut serial]) {
                continue;
            }

            let mut buf = [0u8; 64];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {

                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if 0x61 <= *c && *c <= 0x7a {
                            *c &= !0x20;
                        }
                    }

                    let mut write_offset = 0;
                    while write_offset < count {
                        match serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }

        }
        
    }

    #[task(local = [high1, low1, high2, low2], priority = 1)]
    async fn heater(cx: heater::Context) {

        let high1 = cx.local.high1;
        let low1 = cx.local.low1;
        let high2 = cx.local.high2;
        let low2 = cx.local.low2;

        // lower turn on time during heating (current is too high while wire is still cold)
        for _ in 0..15000 {
            Systick::delay(1.millis()).await;
            high1.set_low();
            low1.set_low();
            high2.set_high();
            low2.set_high();

            Systick::delay(1.millis()).await;
            high1.set_high();
            low1.set_high();
            high2.set_high();
            low2.set_high();
        }

        loop {
            high1.set_low();
            low1.set_low();
            high2.set_high();
            low2.set_high();
            Systick::delay(10.millis()).await;

            high1.set_high();
            low1.set_high();
            high2.set_high();
            low2.set_high();
            Systick::delay(1.millis()).await;

            high1.set_high();
            low1.set_high();
            high2.set_low();
            low2.set_low();
            Systick::delay(10.millis()).await;

            high1.set_high();
            low1.set_high();
            high2.set_high();
            low2.set_high();
            Systick::delay(1.millis()).await;
        }
    }

    #[task(local = [switch, dac], shared = [disp], priority = 1)]
    async fn audio(cx: audio::Context) {

        let switch = cx.local.switch;
        let dac = cx.local.dac;
        let mut disp = cx.shared.disp;

        loop {

            if switch.is_low() {

                disp.lock(|disp| {

                    disp.set_bell_state(true);
                });

                dac.set_dac_fast(PowerDown::Normal, 0x0f0).ok();
                Systick::delay(1.millis()).await;

                dac.set_dac_fast(PowerDown::Normal, 0x000).ok();
                Systick::delay(1.millis()).await;

                disp.lock(|disp| {

                    disp.set_bell_state(false);
                });
            }
            else {
                Systick::delay(10.millis()).await;
            }
        }
    }

    // #[task(shared = [disp], priority = 1)]
    // async fn disp(cx: disp::Context) {

    //     let mut disp = cx.shared.disp;

    //     const NUMBER: [u16; 10] =
    //     [
    //         0123,
    //         1234,
    //         2345,
    //         3456,
    //         4567,
    //         5678,
    //         6789,
    //         7890,
    //         8901,
    //         9012
    //     ];

    //     loop {

    //         for number in NUMBER {

    //             disp.lock(|disp| {

    //                 disp.toggle_dp();
    //                 disp.set_4digits(number).ok();
    //             });

    //             Systick::delay(1000.millis()).await;
    //         }
            
    //     }
    // }

    #[task(local = [heater_voltage, adc], shared = [disp], priority = 1)]
    async fn disp(cx: disp::Context) {

        let heater_voltage = cx.local.heater_voltage;
        let adc = cx.local.adc;
        let mut disp = cx.shared.disp;

        loop {

            let data: u16 = adc.read( heater_voltage).unwrap();
            // let data = 1234;

            disp.lock(|disp| {

                disp.toggle_dp();
                disp.set_4digits(data).ok();
            });

            Systick::delay(100.millis()).await;
        }
    }

    #[task(local = [i2c_bus], shared = [disp], priority = 1)]
    async fn vfd(cx: vfd::Context) {

        let i2c_bus = cx.local.i2c_bus;
        let mut disp = cx.shared.disp;
        
        let high = Pcf8574::new(
            i2c_bus.acquire_i2c(), 
            SlaveAddr::default());
        let low = Pcf8574::new(
            i2c_bus.acquire_i2c(), 
            SlaveAddr::Alternative(false, false, true));

        let mut vfd = VfdDriver::new(high, low);

        loop {
            
            Systick::delay(5.millis()).await;
            
            disp.lock(|disp| {

                vfd.update(disp).ok();
            });
        }
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {

        // rprintln!("idle");

        loop {
            continue;
        }
    }
}
