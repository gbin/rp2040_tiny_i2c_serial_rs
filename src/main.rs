#![no_main]
#![no_std]

use panic_probe as _;
use core::cell::UnsafeCell;
use core::fmt::Write as StdWrite;
use embedded_io::Write as IOWrite;
use fugit::RateExtU32;
use heapless::String;

// use embedded_hal::blocking::i2c::{Write as I2CWrite, WriteRead};
// use embedded_hal::blocking::i2c::Read as I2CRead;

use rp_pico as bsp;
use bsp::entry;
use bsp::hal as hal;

use hal::I2C;
use hal::pac::I2C1;
use hal::pac::UART0;
use hal::gpio::{FunctionI2C, Pin, PullUp, PullDown, FunctionPio0, FunctionUart};
use hal::uart::{DataBits, StopBits, UartConfig, UartPeripheral, Enabled};
use hal::gpio::bank0::{Gpio5, Gpio4, Gpio22, Gpio23, Gpio28, Gpio29};
use hal::pio::PIOExt;
use hal::{clocks::init_clocks_and_plls, pac, usb::UsbBus, watchdog::Watchdog, Clock, Timer};
use hal::{gpio::Pins, Sio};


use usb_device::bus::UsbBusAllocator;
use usb_device::class_prelude::*;
use usb_device::prelude::*;

// use usbd_serial::embedded_io::Write as USBWrite;
// use usbd_serial::SerialPort;

use ws2812_pio::Ws2812;

use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use vl53l0x::VL53L0x;
use smart_leds_trait::RGB8;
use smart_leds_trait::SmartLedsWrite;

// Type definitions to address the screen on the device.
type DisplaySdaPin = Pin<Gpio22, FunctionI2C, PullUp>;
type DisplaySclPin = Pin<Gpio23, FunctionI2C, PullUp>;
type DisplayI2CBus = I2C<I2C1, (DisplaySdaPin, DisplaySclPin)>;
type DisplayI2CInterface = I2CInterface<DisplayI2CBus>;
type DisplaySize = DisplaySize72x40;
type DisplayMode = BufferedGraphicsMode<DisplaySize>;
type Display = Ssd1306<DisplayI2CInterface, DisplaySize, DisplayMode>;

// type definition for the slave i2c bus we want to control from the host
type SlaveSdaPin = Pin<Gpio4, FunctionI2C, PullUp>;
type SlaveSclPin = Pin<Gpio5, FunctionI2C, PullUp>;

// type definitions for the uart console
type UartTxPin = Pin<Gpio28, FunctionUart, PullDown>;
type UartRxPin = Pin<Gpio29, FunctionUart, PullDown>;
type UartConsole = UartPeripheral<Enabled, UART0, (UartTxPin, UartRxPin)>;



static mut SERIAL_CONSOLE: UnsafeCell<Option<UartConsole>> = UnsafeCell::new(None);

// Convenience macro to debug over serial
fn get_console() -> &'static mut UartConsole {
    unsafe {
        // Unsafe block is needed because we're dereferencing a raw pointer
        match SERIAL_CONSOLE.get_mut() {
            Some(serial) => serial,
            None => panic!("SERIAL not initialized"),
        }
    }
}

#[macro_export]
macro_rules! info {
    ($msg:expr) => {{
        let console = get_console();
        let _ = console.write_str(format!("{}\r\n", $msg).as_str());
        let _ = console.flush();
    }};
    ($fmt:expr, $($arg:tt)*) => {{
        let console = get_console();
        let _ = console.write_str(format!($fmt, $($arg)*).as_str());
        let _ = console.write(b"\r\n").unwrap();
        let _ = console.flush();
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
const PRODUCT_ID: u16 =  0xC631;

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
        let _ = self.display.clear(BinaryColor::Off);
    }
}

impl<'a> UsbClass<hal::usb::UsbBus> for TinyI2C<'a> {
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
                info!("F {}", value);
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
                info!("S {}", value);
                xfer.accept(|buf| {
                    buf[0] = TinyI2CStatus::AddressAck as u8;
                Ok(1)}).expect("Errored in accepting request");
            }
            (TinyI2CRequest::IOBeginEnd, flags, addr, length) => {
                info!("I {:x} {:x} {:?}", flags, addr, length);
                xfer.accept(|buf| {
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
// static mut SERIAL: UnsafeCell<Option<SerialPort<UsbBus>>> = UnsafeCell::new(None);

fn get_usb_bus() -> &'static UsbBusAllocator<UsbBus> {
    unsafe {
        // Unsafe block is needed because we're dereferencing a raw pointer
        match &*USB_BUS.get() {
            Some(bus) => bus,
            None => panic!("USB_BUS not initialized"),
        }
    }
}



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

    let sio = Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    
    // Configure the pin 12 for the RGB led with PIO
    let led_pin: Pin<_, FunctionPio0, _> = pins.gpio12.into_function();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut led = Ws2812::new(
        led_pin,
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // Configure pin 28 an 20 for the serial console
    let serial_console_tx_pin: Pin<_, FunctionUart, _> = pins.gpio28.into_function();
    let serial_console_rx_pin: Pin<_, FunctionUart, _> = pins.gpio29.into_function();
    let uart_pins = (
        serial_console_tx_pin,
        serial_console_rx_pin,
    );
    let mut uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS).enable(UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),clocks.peripheral_clock.freq()).unwrap();
    uart.write_str("Pre-info\r\n").unwrap();

    unsafe {
        // Save some global variables: TODO fix this.
        *USB_BUS.get() = Some(UsbBusAllocator::new(UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )));

        *SERIAL_CONSOLE.get() = Some(uart);
    }

    info!("TinyI2C ready");
    let strs = StringDescriptors::new(LangID::EN_US)
        .manufacturer("Guillaume Binet")
        .product("TinyI2c")
        .serial_number("1234");
    let tiny_usb_id = UsbVidPid(VENDOR_ID, PRODUCT_ID);
    let mut usb_dev: UsbDevice<UsbBus> = UsbDeviceBuilder::new(get_usb_bus(), tiny_usb_id)
        .strings(&[strs])
        .unwrap()
        .build();



    let slave_sda_pin: SlaveSdaPin = pins.gpio4.reconfigure();
    let slave_scl_pin: SlaveSclPin = pins.gpio5.reconfigure();
    
    // Configure the second I2c two pins as being I²C, not GPIO
    let i2c_slave = I2C::i2c0(
        pac.I2C0,
        slave_sda_pin,
        slave_scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    
    // Configure two pins as being I²C for the display and Not GPIO
    let display_sda_pin: DisplaySdaPin = pins.gpio22.reconfigure();
    let display_scl_pin: DisplaySclPin = pins.gpio23.reconfigure();

    let i2c_display: DisplayI2CBus = hal::I2C::i2c1(
        pac.I2C1,
        display_sda_pin,
        display_scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let interface: DisplayI2CInterface = I2CDisplayInterface::new(i2c_display);
    let mut display: Display = Ssd1306::new(interface, DisplaySize72x40, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let mut tiny_i2c = TinyI2C::new(get_usb_bus(), display);
    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // delay.delay_ms(3000);
    //
    //
    
    const ORANGE : RGB8 = RGB8::new(255, 165, 0);
    const PINK : RGB8 = RGB8::new(255, 0, 255);
    const RED : RGB8 = RGB8::new(255, 0, 0);
    const GREEN : RGB8 = RGB8::new(0, 255, 0);

    led.write([PINK].iter().cloned()).unwrap();

    let mut tof = VL53L0x::new(i2c_slave).unwrap();
    tof.set_measurement_timing_budget(200000).unwrap();
    tof.start_continuous(0).unwrap();

    led.write([ORANGE].iter().cloned()).unwrap();

    loop {
        //let l = vl.read_range_mm();
        let mls = tof.read_range_continuous_millimeters_blocking();
        if mls.is_ok() {
            let l = mls.unwrap();
            led.write([GREEN].iter().cloned()).unwrap();
            tiny_i2c.clear();
            tiny_i2c.display_3(format!("L: {}", l).as_str());
        } else {
            led.write([RED].iter().cloned()).unwrap();
        }
        // let ok = i2c_slave.write_read(ADDR, &query, &mut response);
        // let ping = i2c_slave.write(ADDR, &[0x0]);
        // if ok.is_ok() {
        //     if response[0] == 0xee {
        //           led.write([GREEN].iter().cloned()).unwrap();
        //     }
        //     else {
        //           led.write([ORANGE].iter().cloned()).unwrap();
        //     }
        // } else {
        //           led.write([RED].iter().cloned()).unwrap();
        // }
        if !usb_dev.poll(&mut [/*get_serial(), */&mut tiny_i2c]) {
            continue;
        }
    }
}

// #[panic_handler]
// fn panic_handler(info: &PanicInfo) -> ! {
//      info!("Panicked with info: {:?} ", info);
//      loop {}
//  }
