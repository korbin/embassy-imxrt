use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::{interrupt, pac, peripherals, Peri, PeripheralType};
use core::future::Future;
use core::marker::PhantomData;
use core::pin::Pin as FuturePin;
use core::task::{Context, Poll};
use embassy_hal_internal::impl_peripheral;
use embassy_hal_internal::interrupt::InterruptExt;
use embassy_sync::waitqueue::AtomicWaker;
use pac::iomuxc::regs::GpioCtl;
use pac::{common::*, IOMUXC};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Level {
    Low,
    High,
}

/// Represents a pull setting for an input.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    None,
    Keep,
    Up22K,
    Up47K,
    Up100K,
    Down100K,
}

/// Drive strength of an output
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Drive {
    Disabled,
    _150R,
    _75R,
    _50R,
    _37R,
    _30R,
    _25R,
    _20R,
}

/// Slew rate of an output
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SlewRate {
    Fast,
    Slow,
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        match level {
            Level::Low => false,
            Level::High => true,
        }
    }
}

pub struct Flex<'d, T: Pin> {
    pub(crate) pin: Peri<'d, T>,
}

impl<'d, T: Pin> Flex<'d, T> {
    #[inline]
    pub fn new(pin: Peri<'d, T>) -> Self {
        use crate::pac::gpio::Gpio;
        let gpio: Gpio = pin.gpio();
        gpio.imr().modify(|r| *r &= !pin.bit());
        // Pin will be in disconnected state.

        Self { pin }
    }

    /// Set the pin's pull.
    #[inline]
    pub fn set_pull(&mut self, pull: Pull) {
        use pac::iomuxc::vals::GpioPus::*;

        self.pin.mux().modify(|r| {
            r.set_mux_mode(0b101.into());
        });

        let ctl = self.pin.ctl();

        let (pke, pue, pus) = match pull {
            Pull::None => (false, false, PUS_0_100K_OHM_PULL_DOWN),
            Pull::Keep => (true, false, PUS_0_100K_OHM_PULL_DOWN),
            Pull::Down100K => (true, true, PUS_0_100K_OHM_PULL_DOWN),
            Pull::Up47K => (true, true, PUS_1_47K_OHM_PULL_UP),
            Pull::Up100K => (true, true, PUS_2_100K_OHM_PULL_UP),
            Pull::Up22K => (true, true, PUS_3_22K_OHM_PULL_UP),
        };

        ctl.modify(|r| {
            r.set_pke(pke);
            r.set_pue(pue);
            r.set_pus(pus);
        });
    }

    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: Drive) {
        let ctl = self.pin.ctl();
        use pac::iomuxc::vals::GpioDse::*;
        use Drive::*;

        ctl.modify(|r| {
            r.set_dse(match strength {
                Disabled => DSE_0_OUTPUT_DRIVER_DISABLED_,
                _150R => DSE_1_R0_150_OHM___3_3V__260_OHM_1_8V__240_OHM_FOR_DDR_,
                _75R => DSE_2_R0_2,
                _50R => DSE_3_R0_3,
                _37R => DSE_4_R0_4,
                _25R => DSE_5_R0_5,
                _20R => DSE_6_R0_6,
                _30R => DSE_7_R0_7,
            })
        });
    }

    // Set the pin's slew rate.
    #[inline]
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        let ctl = self.pin.ctl();
        ctl.modify(|r| r.set_sre(slew_rate == SlewRate::Fast));
    }

    /// Set the pin's Schmitt trigger.
    #[inline]
    pub fn set_schmitt(&mut self, enable: bool) {
        let ctl = self.pin.ctl();
        ctl.modify(|r| r.set_hys(enable));
    }

    /// Put the pin into input mode.
    ///
    /// The pull setting is left unchanged.
    #[inline]
    pub fn set_as_input(&mut self) {
        self.pin.gpio().gdir().modify(|r| *r &= !self.pin.bit())
    }

    #[inline]
    pub fn is_set_as_input(&self) -> bool {
        self.pin.gpio().gdir().read() & self.pin.bit() == 0
    }

    /// Put the pin into output mode.
    ///
    /// The pin level will be whatever was set before (or low by default). If you want it to begin
    /// at a specific level, call `set_high`/`set_low` on the pin first.
    #[inline]
    pub fn set_as_output(&mut self) {
        self.pin.gpio().gdir().modify(|r| *r |= self.pin.bit())
    }

    #[inline]
    pub fn is_set_as_output(&self) -> bool {
        !self.is_set_as_input()
    }

    #[inline]
    pub fn toggle_set_as_output(&mut self) {
        if self.is_set_as_input() {
            self.set_as_output()
        } else {
            self.set_as_input()
        }
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.gpio().dr().read() & self.pin.bit() == 0
    }

    #[inline]
    pub fn get_level(&self) -> Level {
        self.is_high().into()
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.gpio().dr_set().write_value(self.pin.bit())
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.gpio().dr_clear().write_value(self.pin.bit())
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    /// Is the output level high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.is_set_as_output() && self.is_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.is_set_as_output() && self.is_low()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.is_set_high().into()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.gpio().dr_toggle().write_value(self.pin.bit())
    }

    #[inline]
    pub async fn wait_for_high(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::LevelHigh).await;
    }

    #[inline]
    pub async fn wait_for_low(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::LevelLow).await;
    }

    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::EdgeHigh).await;
    }

    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::EdgeLow).await;
    }

    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        InputFuture::new(self.pin.reborrow(), InterruptTrigger::AnyEdge).await;
    }
}

pub struct Input<'d, T: Pin> {
    pub(crate) pin: Flex<'d, T>,
}

impl<'d, T: Pin> Input<'d, T> {
    #[inline]
    pub fn new(pin: Peri<'d, T>, pull: Pull) -> Self {
        let mut pin = Flex::new(pin);
        pin.set_as_input();
        pin.set_pull(pull);
        Self { pin }
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    /// Returns current pin level
    #[inline]
    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await;
    }

    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await;
    }

    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await;
    }

    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await;
    }

    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await;
    }
}

pub struct Output<'d, T: Pin> {
    pin: Flex<'d, T>,
}

impl<'d, T: Pin> Output<'d, T> {
    #[inline]
    pub fn new(pin: Peri<'d, T>, initial_output: Level) -> Self {
        let mut pin = Flex::new(pin);
        match initial_output {
            Level::High => pin.set_high(),
            Level::Low => pin.set_low(),
        }

        pin.set_as_output();
        Self { pin }
    }

    /// Set the pin's drive strength.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: Drive) {
        self.pin.set_drive_strength(strength)
    }

    // Set the pin's slew rate.
    #[inline]
    pub fn set_slew_rate(&mut self, slew_rate: SlewRate) {
        self.pin.set_slew_rate(slew_rate)
    }

    /// Set the output as high.
    #[inline]
    pub fn set_high(&mut self) {
        self.pin.set_high()
    }

    /// Set the output as low.
    #[inline]
    pub fn set_low(&mut self) {
        self.pin.set_low()
    }

    /// Set the output level.
    #[inline]
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level)
    }

    /// Is the output pin set as high?
    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.pin.is_set_high()
    }

    /// Is the output pin set as low?
    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.pin.is_set_low()
    }

    /// What level output is set to
    #[inline]
    pub fn get_output_level(&self) -> Level {
        self.pin.get_output_level()
    }

    /// Toggle pin output
    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }
}

/// Interrupt trigger levels.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptTrigger {
    LevelLow,
    LevelHigh,
    EdgeLow,
    EdgeHigh,
    AnyEdge,
}
pub(crate) unsafe fn init() {
    interrupt::GPIO1_COMBINED_0_15.disable();
    interrupt::GPIO1_COMBINED_0_15.set_priority(interrupt::Priority::P3);
    interrupt::GPIO1_COMBINED_0_15.enable();

    interrupt::GPIO1_COMBINED_16_31.disable();
    interrupt::GPIO1_COMBINED_16_31.set_priority(interrupt::Priority::P3);
    interrupt::GPIO1_COMBINED_16_31.enable();

    interrupt::GPIO2_COMBINED_0_15.disable();
    interrupt::GPIO2_COMBINED_0_15.set_priority(interrupt::Priority::P3);
    interrupt::GPIO2_COMBINED_0_15.enable();

    interrupt::GPIO5_COMBINED_0_15.disable();
    interrupt::GPIO5_COMBINED_0_15.set_priority(interrupt::Priority::P3);
    interrupt::GPIO5_COMBINED_0_15.enable();
}

#[cfg(feature = "rt")]
fn irq_handler<const N: usize>(bank: pac::gpio::Gpio, wakers: &[AtomicWaker; N]) {
    let isr = bank.isr().read();

    for (i, waker) in wakers.iter().enumerate() {
        let bit = 1u32 << i;
        let hit = isr & 1 << i != 0;

        if hit {
            bank.imr().modify(|r| *r &= !bit);

            waker.wake();
        }
    }

    bank.isr().write_value(isr);
}

#[cfg(feature = "rt")]
#[interrupt]
fn GPIO1_COMBINED_0_15() {
    irq_handler(pac::GPIO1, &GPIO1_WAKERS);
}

#[cfg(feature = "rt")]
#[interrupt]
fn GPIO1_COMBINED_16_31() {
    irq_handler(pac::GPIO1, &GPIO1_WAKERS);
}

#[cfg(feature = "rt")]
#[interrupt]
fn GPIO2_COMBINED_0_15() {
    irq_handler(pac::GPIO2, &GPIO2_WAKERS);
}

#[cfg(feature = "rt")]
#[interrupt]
fn GPIO5_COMBINED_0_15() {
    irq_handler(pac::GPIO5, &GPIO5_WAKERS);
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct InputFuture<'a, T: Pin> {
    pin: Peri<'a, T>,
}

impl<'d, T: Pin> InputFuture<'d, T> {
    pub fn new(pin: Peri<'d, T>, level: InterruptTrigger) -> Self {
        use crate::pac::gpio::Gpio;
        use pac::gpio::vals::Icr;
        let gpio: Gpio = pin.gpio();

        let mut edge_sel = gpio.edge_sel().read() & !pin.bit();
        if level == InterruptTrigger::AnyEdge {
            edge_sel |= pin.bit();
        }

        let icr = match level {
            InterruptTrigger::AnyEdge => Icr::FALLING_EDGE,
            InterruptTrigger::EdgeLow => Icr::FALLING_EDGE,
            InterruptTrigger::EdgeHigh => Icr::RISING_EDGE,
            InterruptTrigger::LevelLow => Icr::LOW_LEVEL,
            InterruptTrigger::LevelHigh => Icr::HIGH_LEVEL,
        };

        critical_section::with(|_| {
            gpio.isr().write_value(pin.bit());

            gpio.edge_sel().write_value(edge_sel);

            pin.icr().modify(|r| {
                r.set_pin((pin.pin() % 16) as _, icr);
            });

            gpio.imr().modify(|r| *r |= pin.bit());
        });

        Self { pin }
    }
}

impl<'d, T: Pin> Future for InputFuture<'d, T> {
    type Output = ();

    fn poll(self: FuturePin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let gpio = self.pin.gpio();
        self.pin.waker().register(cx.waker());

        if gpio.imr().read() & self.pin.bit() == 0 {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

pub(crate) mod sealed {
    use super::*;
    use pac::common::*;
    use pac::iomuxc::regs::GpioCtl;

    pub trait Pin: Sized {
        const MASK: u32;

        fn pin(&self) -> u8;

        #[inline]
        fn bit(&self) -> u32 {
            1 << self.pin()
        }

        ////#[inline]
        //fn _pin(&self) -> u8 {
        //    self.pin_gpio() & 0x1f
        //}

        //#[inline]
        //fn _gpio(&self) -> gpio {
        //    match self.pin_gpio() & 0x20 {
        //        #[cfg(feature = "qspi-as-gpio")]
        //        1 => gpio::Qspi,
        //        _ => gpio::gpio0,
        //    }
        //}

        //fn io(&self) -> pac::io::Io {
        //    match self._gpio() {
        //        gpio::gpio0 => crate::pac::IO_gpio0,
        //        #[cfg(feature = "qspi-as-gpio")]
        //        gpio::Qspi => crate::pac::IO_QSPI,
        //    }
        //}

        //fn gpio(&self) -> pac::io::Gpio {
        //    self.io().gpio(self._pin() as _)
        //}

        fn gpio(&self) -> pac::gpio::Gpio;

        fn ctl(&self) -> Reg<GpioCtl, RW>;

        fn mux(&self) -> Reg<pac::iomuxc::regs::GpioMuxCtl00, RW>;

        fn icr(&self) -> Reg<pac::gpio::regs::Icr, RW> {
            self.gpio().icr_x((self.pin() as usize) / 16)
        }

        fn waker(&self) -> &'static AtomicWaker;
        //fn pad_ctrl(&self) -> Reg<pac::gpio, RW> {
        //    let block = match self._gpio() {
        //        gpio::gpio0 => crate::pac::PADS_gpio0,
        //        #[cfg(feature = "qspi-as-gpio")]
        //        gpio::Qspi => crate::pac::PADS_QSPI,
        //    };
        //    block.gpio(self._pin() as _)
        //}

        //fn sio_out(&self) -> pac::sio::Gpio {
        //    SIO.gpio_out(self._gpio() as _)
        //}

        //fn sio_oe(&self) -> pac::sio::Gpio {
        //    SIO.gpio_oe(self._gpio() as _)
        //}

        //fn sio_in(&self) -> Reg<u32, RW> {
        //    SIO.gpio_in(self._gpio() as _)
        //}

        //fn int_proc(&self) -> pac::io::Int {
        //    let proc = SIO.cpuid().read();
        //    self.io().int_proc(proc as _)
        //}
    }
}

pub trait Pin: PeripheralType + sealed::Pin + Sized + 'static {}

macro_rules! impl_pin {
    ($name:ident, $gpio:expr, $ctl:expr, $mux:expr, $waker:expr, $pin_num:expr) => {
        impl Pin for peripherals::$name {}
        impl sealed::Pin for peripherals::$name {
            const MASK: u32 = (1 << $pin_num);

            fn pin(&self) -> u8 {
                $pin_num
            }

            fn gpio(&self) -> pac::gpio::Gpio {
                $gpio
            }

            fn ctl(&self) -> Reg<GpioCtl, RW> {
                $ctl
            }

            fn mux(&self) -> Reg<pac::iomuxc::regs::GpioMuxCtl00, RW> {
                unsafe { core::mem::transmute($mux) }
            }

            fn waker(&self) -> &'static AtomicWaker {
                $waker
            }
        }

        //        impl From<peripherals::$name> for crate::gpio::AnyPin {
        //            fn from(val: peripherals::$name) -> Self {
        //                crate::gpio::Pin::degrade(val)
        //            }
        //        }
    };
}

const NEW_AW: AtomicWaker = AtomicWaker::new();
static GPIO1_WAKERS: [AtomicWaker; 32] = [NEW_AW; 32];
static GPIO2_WAKERS: [AtomicWaker; 16] = [NEW_AW; 16];
static GPIO5_WAKERS: [AtomicWaker; 16] = [NEW_AW; 16];

impl_pin!(
    GPIO_00,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 0),
    IOMUXC.gpio_mux_00(),
    &GPIO1_WAKERS[0],
    0
);

impl_pin!(
    GPIO_01,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 1),
    IOMUXC.gpio_mux_01(),
    &GPIO1_WAKERS[1],
    1
);

impl_pin!(
    GPIO_02,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 2),
    IOMUXC.gpio_mux_02(),
    &GPIO1_WAKERS[2],
    2
);

impl_pin!(
    GPIO_03,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 3),
    IOMUXC.gpio_mux_03(),
    &GPIO1_WAKERS[3],
    3
);

impl_pin!(
    GPIO_04,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 4),
    IOMUXC.gpio_mux_04(),
    &GPIO1_WAKERS[4],
    4
);

impl_pin!(
    GPIO_05,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 5),
    IOMUXC.gpio_mux_05(),
    &GPIO1_WAKERS[5],
    5
);

impl_pin!(
    GPIO_06,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 6),
    IOMUXC.gpio_mux_06(),
    &GPIO1_WAKERS[6],
    6
);

impl_pin!(
    GPIO_07,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 7),
    IOMUXC.gpio_mux_07(),
    &GPIO1_WAKERS[7],
    7
);

impl_pin!(
    GPIO_08,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 8),
    IOMUXC.gpio_mux_08(),
    &GPIO1_WAKERS[8],
    8
);

impl_pin!(
    GPIO_09,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 9),
    IOMUXC.gpio_mux_09(),
    &GPIO1_WAKERS[9],
    9
);

impl_pin!(
    GPIO_10,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 10),
    IOMUXC.gpio_mux_10(),
    &GPIO1_WAKERS[10],
    10
);

impl_pin!(
    GPIO_11,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 11),
    IOMUXC.gpio_mux_11(),
    &GPIO1_WAKERS[11],
    11
);

impl_pin!(
    GPIO_12,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 12),
    IOMUXC.gpio_mux_12(),
    &GPIO1_WAKERS[12],
    12
);

impl_pin!(
    GPIO_13,
    pac::GPIO1,
    IOMUXC.gpio_ctl_x(13 - 13),
    IOMUXC.gpio_mux_13(),
    &GPIO1_WAKERS[13],
    13
);

//GPIO1_COMBINED_0_15,
//GPIO1_COMBINED_16_31,
//GPIO2_COMBINED_0_15,
//GPIO5_COMBINED_0_15,
