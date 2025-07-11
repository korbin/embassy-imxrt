#![no_std]
#![feature(trait_alias)]

pub mod time_driver;
pub use embassy_hal_internal::{Peri, PeripheralType};

pub mod bootloader;
pub mod ccm;
pub mod dma;
pub mod flexio;
pub mod flexspi;
pub mod gpio;
pub mod lpi2c;
//pub mod lpspi;
pub mod lpuart;
pub mod rtwdog;
pub mod usb;

mod imxrt1010;
pub use imxrt1010::*;

#[macro_export]
macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident { $($irq:ident => $($handler:ty),*;)* }) => {
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[no_mangle]
            unsafe extern "C" fn $irq() {
                $(
                    <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();
                )*
            }

            $(
                unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
            )*
        )*
    };
}

macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident { $($irq:ident => $($handler:ty),*;)* }) => {
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[no_mangle]
            unsafe extern "C" fn $irq() {
                $(
                    <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();
                )*
            }

            $(
                unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
            )*
        )*
    };
}

pub fn init() -> Peripherals {
    let peripherals = Peripherals::take();

    pac::CCM.cbcdr().modify(|x| x.set_periph_clk_sel(true));

    while pac::CCM.cdhipr().read().periph_clk_sel_busy() {}

    pac::CCM_ANALOG.pfd_480().modify(|x| x.set_pfd2_frac(12));

    //480*18/24(pfd0)/4
    pac::CCM_ANALOG.pfd_480().modify(|x| x.set_pfd0_frac(24));
    pac::CCM.cscmr1().modify(|x| x.set_flexspi_podf(3.into()));

    // CPU Core
    pac::CCM_ANALOG.pfd_528().modify(|x| x.set_pfd3_frac(18));
    cortex_m::asm::delay(500_000);

    pac::CCM.cbcdr().modify(|x| x.set_periph_clk_sel(false));

    while pac::CCM.cdhipr().read().periph_clk_sel_busy() {}

    pac::CCM
        .cbcmr()
        .write(|v| v.set_pre_periph_clk_sel(pac::ccm::vals::PrePeriphClkSel::PRE_PERIPH_CLK_SEL_0));

    loop {
        if !pac::CCM_ANALOG.pll_usb1().read().enable() {
            pac::CCM_ANALOG.pll_usb1_set().write(|v| v.set_enable(true));
            continue;
        }

        if !pac::CCM_ANALOG.pll_usb1().read().power() {
            pac::CCM_ANALOG.pll_usb1_set().write(|v| v.set_power(true));
            continue;
        }

        if !pac::CCM_ANALOG.pll_usb1().read().lock() {
            continue;
        }

        if pac::CCM_ANALOG.pll_usb1().read().bypass() {
            pac::CCM_ANALOG.pll_usb1_clr().write(|v| v.set_bypass(true));
            continue;
        }

        if !pac::CCM_ANALOG.pll_usb1().read().en_usb_clks() {
            pac::CCM_ANALOG
                .pll_usb1_set()
                .write(|v| v.set_en_usb_clks(true));
            continue;
        }
        break;
    }

    pac::CCM.ccgr6().modify(|v| v.set_cg0(1));

    unsafe {
        dma::init();
        time_driver::init();
        gpio::init();
    };

    peripherals
}

pub fn unique_id() -> u64 {
    let lo = pac::OCOTP.hw_ocotp_cfg0().read() as u64;
    let hi = (pac::OCOTP.hw_ocotp_cfg1().read() as u64) << 32;

    hi | lo
}
