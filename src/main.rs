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
        i2c::{self, *},
        spi::{self, *},
    };
use rtic_monotonics::systick::*;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use pcf857x::{Pcf8574, SlaveAddr};
use mcp4725::*;
use shared_bus;
use embedded_sdmmc::*;
use bme280::i2c::BME280;

use rata::vfd_driver::{VfdDriver, VfdControl};
use rata::button::ThreeButtons;


#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [CAN_RX1])]
mod app {
    use super::*;


    #[shared]
    struct Shared {
        disp: VfdControl,
        adc: Adc<you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::ADC1>,
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
        dac: MCP4725<BlockingI2c<you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::I2C2, (Pin<'B', 10, Alternate<OpenDrain>>, Pin<'B', 11, Alternate<OpenDrain>>)>>,
        i2c_bus: shared_bus::BusManager<shared_bus::NullMutex<BlockingI2c<you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::I2C1, (Pin<'B', 8, Alternate<OpenDrain>>, Pin<'B', 9, Alternate<OpenDrain>>)>>>,
        light_sensor: Pin<'A', 0, Analog>,
        button_col: (Pin<'A', 5>, Pin<'A', 6>, Pin<'A', 7>),
        button_row: (Pin<'B', 12, Output>, Pin<'B', 2, Output>, Pin<'B', 0, Output>),
        delay: stm32f1xx_hal::timer::Delay<you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::TIM2, 1000000>,
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
        let (pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
        let spi_cs = pa15.into_push_pull_output(&mut gpioa.crh);
        let spi_pins = (
            pb3.into_alternate_push_pull(&mut gpiob.crl),
            pb4.into_floating_input(&mut gpiob.crl),
            gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl)
        );
        let light_sensor = gpioa.pa0.into_analog(&mut gpioa.crl);
        let button_col = (
            gpioa.pa5.into_floating_input(&mut gpioa.crl),
            gpioa.pa6.into_floating_input(&mut gpioa.crl),
            gpioa.pa7.into_floating_input(&mut gpioa.crl),
        );
        let button_row = (
            gpiob.pb12.into_push_pull_output_with_state(&mut gpiob.crh, High),
            gpiob.pb2.into_push_pull_output_with_state(&mut gpiob.crl, High),
            gpiob.pb0.into_push_pull_output_with_state(&mut gpiob.crl, High),
        );

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
            i2c::Mode::Fast {
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
            i2c::Mode::Fast {
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

        let mut disp = VfdControl::new();

        // let spi_mode = spi::Mode {
        //     polarity: Polarity::IdleLow,
        //     phase: Phase::CaptureOnFirstTransition,
        // };
        // let spi = Spi::spi1(
        //     cx.device.SPI1, spi_pins, &mut afio.mapr, spi_mode, 100.kHz(), clocks);
        // let sd_card = SdCard::new(
        //     spi, spi_cs, delay);
        // let volume = match sd_card.num_bytes() {
        //     Ok(volume) => volume / 10_000,
        //     Err(_) => 0
        // };
        // disp.set_4digits(volume as u16).ok();


        beep::spawn().ok();
        // usb::spawn().ok();
        heater::spawn().ok();
        audio::spawn().ok();
        // voltage::spawn().ok();
        vfd::spawn().ok();
        // light::spawn().ok();
        button::spawn().ok();

        (
            Shared {
                disp,
                adc,
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
                dac,
                i2c_bus,
                light_sensor,
                button_col,
                button_row,
                delay
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

    #[task(shared = [disp, adc], local = [heater_voltage], priority = 1)]
    async fn voltage(cx: voltage::Context) {

        let heater_voltage = cx.local.heater_voltage;
        let mut adc = cx.shared.adc;
        let mut disp = cx.shared.disp;

        loop {
            
            let data = adc.lock(|adc| {

                adc.read( heater_voltage).unwrap()
            });

            disp.lock(|disp| {

                disp.toggle_dp();
                disp.set_4digits(data).ok();
            });

            Systick::delay(100.millis()).await;
        }
    }

    #[task(shared = [disp, adc], local = [light_sensor], priority = 1)]
    async fn light(cx: light::Context) {

        let light = cx.local.light_sensor;
        let mut adc = cx.shared.adc;
        let mut disp = cx.shared.disp;

        loop {
            
            let data = adc.lock(|adc| {

                adc.read( light).unwrap()
            });

            disp.lock(|disp| {

                disp.toggle_dp();
                disp.set_4digits(data).ok();
            });

            Systick::delay(100.millis()).await;
        }
    }

    #[task(shared = [disp], local = [button_col, button_row], priority = 1)]
    async fn button(cx: button::Context) {

        let btn_col = cx.local.button_col;
        let btn_row = cx.local.button_row;
        let mut disp = cx.shared.disp;

        let mut btn147 = ThreeButtons::new(
            (&btn_col.0, &btn_col.1, &btn_col.2), &mut btn_row.0);
        let mut btn258 = ThreeButtons::new(
            (&btn_col.0, &btn_col.1, &btn_col.2), &mut btn_row.1);
        let mut btn369 = ThreeButtons::new(
            (&btn_col.0, &btn_col.1, &btn_col.2), &mut btn_row.2);

        let mut pressed = [false; 10];

        loop {
            
            btn147.prepare();
            Systick::delay(1.millis()).await;
            (pressed[1], pressed[4], pressed[7]) = btn147.is_pressed();
            btn258.prepare();
            Systick::delay(1.millis()).await;
            (pressed[2], pressed[5], pressed[8]) = btn258.is_pressed();
            btn369.prepare();
            Systick::delay(1.millis()).await;
            (pressed[3], pressed[6], pressed[9]) = btn369.is_pressed();

            let mut number = 0;
            for (index, pressed) in pressed.iter().enumerate() {
                if *pressed {
                    number = index as u16;
                }
            };

            disp.lock(|disp| {

                disp.toggle_dp();
                disp.set_4digits(number).ok();
            });

            Systick::delay(10.millis()).await;
        }
    }

    #[task(shared = [disp], local = [i2c_bus, delay], priority = 1)]
    async fn vfd(cx: vfd::Context) {

        let i2c_bus = cx.local.i2c_bus;
        let mut delay = cx.local.delay;
        let mut disp = cx.shared.disp;
        
        let high = Pcf8574::new(
            i2c_bus.acquire_i2c(), 
            SlaveAddr::default());
        let low = Pcf8574::new(
            i2c_bus.acquire_i2c(), 
            SlaveAddr::Alternative(false, false, true));

        // let temp_sensor = BME280::new_primary(
        //     i2c_bus.acquire_i2c(), 
        //     *delay);
        

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
