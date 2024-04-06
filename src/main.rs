#![no_std]
#![no_main]
use core::panic::PanicInfo;
use fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin, PullUp};
use hal::{clocks::init_clocks_and_plls, pac, usb::UsbBus, watchdog::Watchdog};
use hal::{gpio::Pins, Sio};
use rp2040_hal as hal;
use rp_pico::entry;
use usb_device::bus::UsbBusAllocator;
use usb_device::class_prelude::*;
use usb_device::prelude::*;
// use usbd_serial::SerialPort;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const TUSB_CLASS_MISC: u8 = 0xEF;

struct TinyI2C {
    // fields for your custom class
}

impl TinyI2C {
    fn new(_alloc: &UsbBusAllocator<UsbBus>) -> TinyI2C {
        TinyI2C {}
    }
}

impl UsbClass<rp2040_hal::usb::UsbBus> for TinyI2C {
    // Called after a USB reset after the bus reset sequence is complete.
    fn reset(&mut self) {}

    // Called whenever the `UsbDevice` is polled.
    fn poll(&mut self) {}
}

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[entry]
unsafe fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let tiny_usb_id = UsbVidPid(0x0403, 0xC631);
    // alternate: let tiny_usb_id = UsbVidPid(0x1c40, 0x0534);
    // let mut cdc_dev = SerialPort::new(&usb_bus);
    let mut tiny_i2c = TinyI2C::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, tiny_usb_id)
        .device_class(TUSB_CLASS_MISC)
        .manufacturer("Guillaume Binet")
        .product("I2C adapter")
        .serial_number("12345678")
        .build();

    let sio = Sio::new(pac.SIO);
    // Set the pins to their default state
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio22.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio23.reconfigure();

    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize72x40, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();
    loop {
        // if usb_dev.poll(&mut [&mut cdc_dev, &mut tiny_i2c]) {}
        if usb_dev.poll(&mut [&mut tiny_i2c]) {

        }
    }
}

#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> ! {
    loop {}
}
