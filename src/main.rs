#![no_std]
#![no_main]
use core::cell::UnsafeCell;
use core::fmt::Write as _;
use core::panic::PanicInfo;
use embedded_hal::blocking::delay::DelayMs;
use fugit::RateExtU32;
use hal::gpio::{FunctionI2C, Pin, PullUp};
use hal::{clocks::init_clocks_and_plls, pac, usb::UsbBus, watchdog::Watchdog, Clock};
use hal::{gpio::Pins, Sio};
use heapless::String;
use rp2040_hal as hal;
use rp_pico::entry;
use usb_device::bus::UsbBusAllocator;
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::embedded_io::Write;
use usbd_serial::SerialPort;

use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use rp2040_hal::gpio::bank0::{Gpio22, Gpio23};
use rp2040_hal::pac::I2C1;
use rp2040_hal::I2C;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

// Type definitions to address the screen on the device.
type DisplaySdaPin = Pin<Gpio22, FunctionI2C, PullUp>;
type DisplaySclPin = Pin<Gpio23, FunctionI2C, PullUp>;
type DisplayI2CBus = I2C<I2C1, (DisplaySdaPin, DisplaySclPin)>;
type DisplayI2CInterface = I2CInterface<DisplayI2CBus>;
type DisplaySize = DisplaySize72x40;
type DisplayMode = BufferedGraphicsMode<DisplaySize>;
type Display = Ssd1306<DisplayI2CInterface, DisplaySize, DisplayMode>;

// Convenience macro to debug over serial
#[macro_export]
macro_rules! info {
    ($msg:expr) => {{
        let serial = get_serial();
        let _ = write!(serial, "{}\r\n", $msg);
        let _ = serial.flush();
    }};
    ($fmt:expr, $($arg:tt)*) => {{
        let serial = get_serial();
        let _ = write!(serial, $fmt, $($arg)*);
        serial.write(b"\r\n").unwrap();
        let _ = serial.flush();
    }};
}

macro_rules! format {
    ($fmt:expr, $($arg:tt)*) => {{
        let mut s = String::<32>::new();
        let _ = write!(s, $fmt, $($arg)*);
        s
    }};
    ($msg:expr) => {{
        let mut s = String::<32>::new();
        let _ = write!(s, "{}", $msg);
        s
    }};
}

const USB_CLASS_CDC: u8 = 0x02;
const VENDOR_ID: u16 = 0x0403;

bitflags::bitflags! {
    pub struct I2CFunc: u32 {
        const I2C = 0x00000001;
        const BIT_ADDR_10 = 0x00000002;
        const PROTOCOL_MANGLING = 0x00000004; // I2C_M_{REV_DIR_ADDR,NOSTART,..}
        const SMBUS_HWPEC_CALC = 0x00000008; // SMBus 2.0
        const SMBUS_READ_WORD_DATA_PEC = 0x00000800; // SMBus 2.0
        const SMBUS_WRITE_WORD_DATA_PEC = 0x00001000; // SMBus 2.0
        const SMBUS_PROC_CALL_PEC = 0x00002000; // SMBus 2.0
        const SMBUS_BLOCK_PROC_CALL_PEC = 0x00004000; // SMBus 2.0
        const SMBUS_BLOCK_PROC_CALL = 0x00008000; // SMBus 2.0
        const SMBUS_QUICK = 0x00010000;
        const SMBUS_READ_BYTE = 0x00020000;
        const SMBUS_WRITE_BYTE = 0x00040000;
        const SMBUS_READ_BYTE_DATA = 0x00080000;
        const SMBUS_WRITE_BYTE_DATA = 0x00100000;
        const SMBUS_READ_WORD_DATA = 0x00200000;
        const SMBUS_WRITE_WORD_DATA = 0x00400000;
        const SMBUS_PROC_CALL = 0x00800000;
        const SMBUS_READ_BLOCK_DATA = 0x01000000;
        const SMBUS_WRITE_BLOCK_DATA = 0x02000000;
        const SMBUS_READ_I2C_BLOCK = 0x04000000; // I2C-like block transfer
        const SMBUS_WRITE_I2C_BLOCK = 0x08000000; // w/ 1-byte reg. addr.
        const SMBUS_READ_I2C_BLOCK_2 = 0x10000000; // I2C-like block transfer
        const SMBUS_WRITE_I2C_BLOCK_2 = 0x20000000; // w/ 2-byte reg. addr.
        const SMBUS_READ_BLOCK_DATA_PEC = 0x40000000; // SMBus 2.0
        const SMBUS_WRITE_BLOCK_DATA_PEC = 0x80000000; // SMBus 2.0

        // Derived funcs
        const SMBUS_BYTE = I2CFunc::SMBUS_READ_BYTE.bits() | I2CFunc::SMBUS_WRITE_BYTE.bits();
        const SMBUS_BYTE_DATA = I2CFunc::SMBUS_READ_BYTE_DATA.bits() | I2CFunc::SMBUS_WRITE_BYTE_DATA.bits();
        const SMBUS_WORD_DATA = I2CFunc::SMBUS_READ_WORD_DATA.bits() | I2CFunc::SMBUS_WRITE_WORD_DATA.bits();
        const SMBUS_BLOCK_DATA = I2CFunc::SMBUS_READ_BLOCK_DATA.bits() | I2CFunc::SMBUS_WRITE_BLOCK_DATA.bits();
        const SMBUS_I2C_BLOCK = I2CFunc::SMBUS_READ_I2C_BLOCK.bits() | I2CFunc::SMBUS_WRITE_I2C_BLOCK.bits();
        const SMBUS_EMUL= I2CFunc::SMBUS_QUICK.bits()
        | I2CFunc::SMBUS_BYTE.bits()
        | I2CFunc::SMBUS_BYTE_DATA.bits()
        | I2CFunc::SMBUS_WORD_DATA.bits()
        | I2CFunc::SMBUS_PROC_CALL.bits()
        | I2CFunc::SMBUS_WRITE_BLOCK_DATA.bits()
        | I2CFunc::SMBUS_WRITE_BLOCK_DATA_PEC.bits()
        | I2CFunc::SMBUS_I2C_BLOCK.bits();

        // The current supported configuration
        const SUPPORTED = I2CFunc::I2C.bits() | I2CFunc::SMBUS_EMUL.bits();
    }
}

enum TinyI2CRequest {
    Echo,
    GetFunc,
    SetDelay,
    GetStatus,
    IO,
    IOBegin,
    IOEnd,
    IOBeginEnd,
}

impl TinyI2CRequest {
    fn from_ordinal(ordinal: u8) -> Option<Self> {
        match ordinal {
            0 => Some(Self::Echo),
            1 => Some(Self::GetFunc),
            2 => Some(Self::SetDelay),
            3 => Some(Self::GetStatus),
            4 => Some(Self::IO),
            5 => Some(Self::IOBegin),
            6 => Some(Self::IOEnd),
            7 => Some(Self::IOBeginEnd),
            _ => None,
        }
    }
}

enum TinyI2CStatus {
    Idle,
    AddressAck,
    AddressNak,
}

impl TinyI2CStatus {
    fn from_ordinal(ordinal: u8) -> Option<Self> {
        match ordinal {
            0 => Some(Self::Idle),
            1 => Some(Self::AddressAck),
            2 => Some(Self::AddressNak),
            _ => None,
        }
    }
}

struct TinyI2C<'a> {
    display: Display,
    text_style: MonoTextStyle<'a, BinaryColor>,
}

impl<'a> TinyI2C<'a> {
    fn new(_alloc: &UsbBusAllocator<UsbBus>, mut display: Display) -> TinyI2C {
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

        display.flush().unwrap();
        let mut obj = TinyI2C {
            display,
            text_style,
        };
        obj.display_1("TinyI2C ready");
        obj
    }

    fn display_1(&mut self, msg: &str) {
        Text::with_baseline(msg, Point::zero(), self.text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();
        self.display.flush().unwrap();
    }
    fn display_2(&mut self, msg: &str) {
        Text::with_baseline(msg, Point::new(0, 10), self.text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();
        self.display.flush().unwrap();
    }

    fn display_3(&mut self, msg: &str) {
        Text::with_baseline(msg, Point::new(0, 20), self.text_style, Baseline::Top)
            .draw(&mut self.display)
            .unwrap();
        self.display.flush().unwrap();
    }

    fn clear(&mut self) {
        self.display.clear(BinaryColor::Off);
    }
}

impl<'a> UsbClass<rp2040_hal::usb::UsbBus> for TinyI2C<'a> {
    // Called after a USB reset after the bus reset sequence is complete.
    fn reset(&mut self) {
        // self.display_secondary("USB reset");
    }

    // Called whenever the `UsbDevice` is polled.
    fn poll(&mut self) {
        //self.display_secondary("USB poll");
    }

    // out == host to device
    fn control_out(&mut self, xfer: ControlOut<'_, '_, '_, UsbBus>) {
        let req = xfer.request();
         // self.display_1(format!("CO:{:x}", req.request).as_str());
        // self.display_1(format!("T:{:?}", req.request_type).as_str());
        // self.display_2(format!("V:{:x}", req.value).as_str());
        // self.display_3(format!("I:{:x}", req.index).as_str());
        if req.request_type != control::RequestType::Vendor {
            return;
        }
        let i2creq = if let Some(i2creq) = TinyI2CRequest::from_ordinal(req.request) {
            i2creq
        } else {
            return;
        };
        match (i2creq, req.value, req.index, req.length) {
            (TinyI2CRequest::SetDelay, value, _, _) => {
                //info!("Set delay to {}", delay);
                self.clear();
                self.display_2(format!("Delay {}", value).as_str());
                xfer.accept().unwrap();
            }
            (TinyI2CRequest::Echo, value, _, _) => {
                // info!("Echo {}", data[1]);
                self.display_2(format!("E {}", value).as_str());
                xfer.accept().unwrap();
            }
            (TinyI2CRequest::IOBeginEnd, flags, addr, length) => {
                // info!("I2C IO: addr: 0x{:02x}, len: {}, data: {:?}", value, i2c_len, i2c_data);
                self.display_3(format!("O {:x} {:x} {:?}", flags, addr, length).as_str());
                xfer.accept().unwrap();
            }
            _ => {}
        }
    }

    // in == device to host
    fn control_in(&mut self, xfer: ControlIn<'_, '_, '_, UsbBus>) {
        // Match on the setup packet to identify the request.
        let req = xfer.request();
        self.clear();
        self.display_1(format!("CI:{:x}", req.request).as_str());

        if req.request_type != control::RequestType::Vendor {
            return;
        }
        let i2creq = if let Some(i2creq) = TinyI2CRequest::from_ordinal(req.request) {
            i2creq
        } else {
            return;
        };
        match (i2creq, req.value, req.index, req.length) {
            (TinyI2CRequest::GetFunc, value, _, _) => {
                // info!("GetFunc");
                xfer.accept(|buf| {
                    if buf.len() < 4 {
                        return Err(UsbError::BufferOverflow);
                    }
                    let bytes = I2CFunc::SUPPORTED.bits().to_ne_bytes();
                    buf[..4].copy_from_slice(&bytes);
                    Ok(4)
                }).expect("Errored in accepting request");
            }
            (TinyI2CRequest::GetStatus, value, _, _) => {
                //info!("GetStatus");
                // self.display_3(format!("S {}", value).as_str());
                xfer.accept(|buf| {
                    buf[0] = TinyI2CStatus::AddressAck as u8;
                Ok(1)}).expect("Errored in accepting request");;
            }
            (TinyI2CRequest::IOBeginEnd, flags, addr, length) => {
                // info!("I2C IO: addr: 0x{:02x}, len: {}, data: {:?}", value, i2c_len, i2c_data);
                xfer.accept(|buf| {
                    //self.display_3(format!("I {:x} {:x} {:?}", flags, addr, length).as_str());
                    buf[0] = 0x01;
                    Ok(length as usize)
                }).expect("Errored in accepting request");
            }
            _ => {}
        }
    }
}

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

static mut USB_BUS: UnsafeCell<Option<UsbBusAllocator<UsbBus>>> = UnsafeCell::new(None);
static mut SERIAL: UnsafeCell<Option<SerialPort<UsbBus>>> = UnsafeCell::new(None);

fn get_usb_bus() -> &'static UsbBusAllocator<UsbBus> {
    unsafe {
        // Unsafe block is needed because we're dereferencing a raw pointer
        match &*USB_BUS.get() {
            Some(bus) => bus,
            None => panic!("USB_BUS not initialized"),
        }
    }
}

fn get_serial() -> &'static mut SerialPort<'static, UsbBus> {
    unsafe {
        // Unsafe block is needed because we're dereferencing a raw pointer
        match SERIAL.get_mut() {
            Some(serial) => serial,
            None => panic!("SERIAL not initialized"),
        }
    }
}
#[entry]
unsafe fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

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

    unsafe {
        *USB_BUS.get() = Some(UsbBusAllocator::new(UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )));

        *SERIAL.get() = Some(SerialPort::new(get_usb_bus()));
    }

    let strs = StringDescriptors::new(LangID::EN_US)
        .manufacturer("Guillaume Binet")
        .product("TinyI2c")
        .serial_number("1234");
    let tiny_usb_id = UsbVidPid(0x0403, 0xC631);
    let mut usb_dev: UsbDevice<UsbBus> = UsbDeviceBuilder::new(get_usb_bus(), tiny_usb_id)
        .strings(&[strs])
        .unwrap()
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
    let sda_pin: DisplaySdaPin = pins.gpio22.reconfigure();
    let scl_pin: DisplaySclPin = pins.gpio23.reconfigure();

    let i2c: DisplayI2CBus = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let interface: DisplayI2CInterface = I2CDisplayInterface::new(i2c);
    let mut display: Display = Ssd1306::new(interface, DisplaySize72x40, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let mut tiny_i2c = TinyI2C::new(get_usb_bus(), display);
    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // delay.delay_ms(3000);
    info!("TinyI2C ready");
    loop {
        if !usb_dev.poll(&mut [get_serial(), &mut tiny_i2c]) {
            continue;
        }
    }
}

#[panic_handler]
fn panic_handler(_info: &PanicInfo) -> ! {
    info!("Panicked ... ");
    loop {}
}
