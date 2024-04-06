#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rp2040_hal::{clocks::init_clocks_and_plls, pac, usb::UsbBus, watchdog::Watchdog};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usb_device::class_prelude::*; 
use usbd_serial::SerialPort;
use rp_pico::entry;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates

struct TinyI2C {
    // fields for your custom class
}

impl TinyI2C {
    fn new(alloc: &UsbBusAllocator<UsbBus>) -> TinyI2C {
        TinyI2C {
        }
    }
}

impl UsbClass<rp2040_hal::usb::UsbBus> for TinyI2C {

    /// Called after a USB reset after the bus reset sequence is complete.
    fn reset(&mut self) {}

    /// Called whenever the `UsbDevice` is polled.
    fn poll(&mut self) {

    }
}

#[entry]
unsafe fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog
    ).ok().unwrap();


    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let tiny_usb_id = UsbVidPid(0x0403, 0xC631);
    // alternate: let tiny_usb_id = UsbVidPid(0x1c40, 0x0534);
    let mut cdc_dev = SerialPort::new(&usb_bus);
    let mut tiny_i2c = TinyI2C::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, tiny_usb_id)
    .manufacturer("Guillaume Binet")
    .product("I2C adapter")
    .serial_number("12345678")
    .device_class(0x02) // CDC
    .build();

    loop {
        if usb_dev.poll(&mut [&mut cdc_dev, &mut tiny_i2c]) {
        }

    }
}


#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> ! {
    loop {}
}
