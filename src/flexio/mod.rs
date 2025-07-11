use crate::{
    dma::{self, Channel, Transfer, Word},
    interrupt, pac, peripherals, Peri,
};
use bitvec::prelude::*;
use core::sync::atomic::{compiler_fence, Ordering};

use core::marker::PhantomData;

pub struct InterruptHandler {}

impl<T: interrupt::typelevel::Interrupt> interrupt::typelevel::Handler<T> for InterruptHandler {
    unsafe fn on_interrupt() {
        //
    }
}

pub struct Flexio<'d, T: Instance> {
    _phantom: &'d PhantomData<T>,
}

impl<'d, T: Instance> Flexio<'d, T> {
    pub fn new() -> Self {
        pac::CCM.ccgr5().modify(|v| v.set_cg1(0));

        pac::CCM
            .cscmr2()
            .modify(|v| v.set_flexio1_clk_sel(pac::ccm::vals::Flexio1clkSel::FLEXIO1_CLK_SEL_1));

        pac::CCM.cs1cdr().modify(|v| {
            v.set_flexio1_clk_pred(0x01.into());
            v.set_flexio1_clk_podf(0x00.into());
        });

        pac::CCM.ccgr5().modify(|v| v.set_cg1(1));

        Self {
            _phantom: &PhantomData,
        }
    }

    pub fn enable(&mut self) {
        T::regs().ctrl().modify(|v| {
            v.set_flexen(true);
            v.set_fastacc(true);
            v.set_swrst(false);
        });
    }

    pub fn disable(&mut self) {
        T::regs().ctrl().modify(|v| {
            v.set_flexen(false);
            v.set_swrst(false);
            v.set_fastacc(true);
        });
    }

    pub fn set_fastacc(&mut self, enable: bool) {
        T::regs().ctrl().modify(|v| {
            v.set_fastacc(enable);
        });
    }

    pub fn shifter<'a>(&'a self, n: u8) -> Shifter<'a, T> {
        Shifter {
            _phantom: &PhantomData,
            instance: n,
        }
    }

    pub fn timer<'a>(&'a self, n: u8) -> Timer<'a, T> {
        Timer {
            _phantom: &PhantomData,
            instance: n,
        }
    }
}

#[derive(Debug, Default, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TimerPolarity {
    #[default]
    PosEdge = 0,
    NegEdge = 1,
}

#[derive(Debug, Default, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PinPolarity {
    #[default]
    ActiveHigh = 0,
    ActiveLow = 1,
}

#[derive(Debug, Default, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ShifterMode {
    #[default]
    Disabled = 0b000,
    Receive = 0b001,
    Transmit = 0b010,
    Reserved = 0b011,
    MatchStore = 0b100,
    MatchContinuous = 0b101,
    State = 0b110,
    Logic = 0b111,
}

#[derive(Debug, Default, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ShifterPinConfiguration {
    #[default]
    Disabled = 0b00,
    OpenDrain = 0b01,
    Bidirectional = 0b10,
    Output = 0b11,
}

pub struct Shifter<'d, T: Instance> {
    _phantom: &'d PhantomData<T>,
    instance: u8,
}

impl<'d, T: Instance> Shifter<'d, T> {
    pub fn set_mode(&mut self, value: ShifterMode) {
        T::regs().shiftctl(self.instance as usize).modify(|v| {
            v.set_smod(pac::flexio::vals::Smod::from_bits(value as u8));
        });
    }

    pub fn set_timer(&mut self, timer: &Timer<'d, T>) {
        T::regs()
            .shiftctl(self.instance as usize)
            .modify(|v| v.set_timsel(timer.instance));
    }

    pub fn set_pin_configuration(&mut self, value: ShifterPinConfiguration) {
        let value = pac::flexio::vals::ShiftctlPincfg::from_bits(value as u8);

        T::regs()
            .shiftctl(self.instance as usize)
            .modify(|v| v.set_pincfg(value));
    }

    pub fn set_pin(&mut self, value: u8) {
        let shiftctl = T::regs().shiftctl(self.instance as usize);
        shiftctl.modify(|v| v.set_pinsel(value));
    }

    pub fn set_pin_polarity(&mut self, value: bool) {
        T::regs()
            .shiftctl(self.instance as usize)
            .modify(|v| v.set_pinpol(value));
    }

    pub fn set_timer_polarity(&mut self, value: bool) {
        T::regs()
            .shiftctl(self.instance as usize)
            .modify(|v| v.set_timpol(value));
    }

    pub fn set_parallel_width(&mut self, value: u8) {
        T::regs()
            .shiftcfg(self.instance as usize)
            .modify(|v| v.set_pwidth(value));
    }

    pub fn enable_dma(&mut self) {
        T::regs()
            .shiftsden()
            .modify(|r| r.set_ssde(r.ssde() | (1 << self.instance)));
    }

    pub fn disable_dma(&mut self) {
        T::regs()
            .shiftsden()
            .modify(|r| r.set_ssde(r.ssde() & !(1 << self.instance)));
    }

    #[inline]
    pub fn read_nonblocking(&mut self) -> u32 {
        compiler_fence(Ordering::SeqCst);
        let res = T::regs().shiftbuf(self.instance as usize).read();
        compiler_fence(Ordering::SeqCst);
        res
    }

    pub fn write_blocking(&mut self, data: u32) {
        compiler_fence(Ordering::SeqCst);
        T::regs().shiftbuf(self.instance as usize).write_value(data);

        compiler_fence(Ordering::SeqCst);
        while !T::regs().shiftstat().read().ssf().view_bits::<Lsb0>()[self.instance as usize] {}
        compiler_fence(Ordering::SeqCst);
    }

    #[inline]
    pub fn write_nonblocking(&mut self, data: u32) {
        compiler_fence(Ordering::SeqCst);
        T::regs().shiftbuf(self.instance as usize).write_value(data);
        compiler_fence(Ordering::SeqCst);
    }

    pub async fn write_async(&mut self, data: u32) {
        compiler_fence(Ordering::SeqCst);
        T::regs().shiftbuf(self.instance as usize).write_value(data);

        compiler_fence(Ordering::SeqCst);
        while !T::regs().shiftstat().read().ssf().view_bits::<Lsb0>()[self.instance as usize] {
            embassy_futures::yield_now().await;
        }
        compiler_fence(Ordering::SeqCst);
    }

    pub async fn write_dma<C: Channel, W: Word>(&mut self, buffer: &[W], tx_dma: Peri<'_, C>) {
        compiler_fence(Ordering::SeqCst);
        unsafe { crate::dma::write(tx_dma, buffer, self).await }
        compiler_fence(Ordering::SeqCst);
    }

    pub fn setup_dma<'a, C: Channel>(
        &mut self,
        buffer: &[u32],
        tx_dma: Peri<'a, C>,
    ) -> Transfer<'a, C> {
        unsafe { crate::dma::write(tx_dma, buffer, self) }
    }
}

impl<'d, W: dma::Word, T: Instance> dma::Destination<W> for Shifter<'d, T> {
    fn destination_signal(&self) -> u8 {
        match self.instance {
            0 | 1 => 0,
            2 | 3 => 64,
            4 | 5 => 1,
            6 | 7 => 65,
            _ => 0,
        }
    }

    fn disable_destination(&mut self) {
        compiler_fence(Ordering::SeqCst);
        self.disable_dma();
    }

    fn enable_destination(&mut self) {
        compiler_fence(Ordering::SeqCst);
        self.enable_dma();
    }
    fn destination_address(&self) -> *const W {
        compiler_fence(Ordering::SeqCst);
        T::regs().shiftbuf(self.instance as usize).as_ptr() as *const _
    }
    fn destination_type(&self) -> dma::DmaType {
        compiler_fence(Ordering::SeqCst);
        dma::DmaType::Hardware
    }
}

pub struct Timer<'d, T: Instance> {
    _phantom: &'d PhantomData<T>,
    instance: u8,
}

impl<'d, T: Instance> Timer<'d, T> {
    #[inline]
    pub fn set_compare(&mut self, value: u16) {
        compiler_fence(Ordering::SeqCst);

        T::regs()
            .timcmp(self.instance.into())
            .write(|v| v.set_cmp(value));

        compiler_fence(Ordering::SeqCst);
    }

    #[inline]
    pub fn set_compare_hilo(&mut self, hi: u8, lo: u8) {
        compiler_fence(Ordering::SeqCst);
        self.set_compare((hi as u16) << 8 | lo as u16);
    }

    #[inline]
    pub fn set_enable_condition(&mut self, value: u8) {
        T::regs()
            .timcfg(self.instance.into())
            .modify(|v| v.set_timena(value.into()));
    }

    #[inline]
    pub fn set_disable_condition(&mut self, value: u8) {
        T::regs()
            .timcfg(self.instance.into())
            .modify(|v| v.set_timdis(value.into()));
    }

    #[inline]
    pub fn set_reset_condition(&mut self, value: u8) {
        T::regs()
            .timcfg(self.instance.into())
            .modify(|v| v.set_timrst(value.into()));
    }

    #[inline]
    pub fn set_trigger_source(&mut self, internal: bool) {
        T::regs()
            .timctl(self.instance.into())
            .modify(|v| v.set_trgsrc(internal));
    }

    #[inline]
    pub fn set_trigger_sel(&mut self, value: u8) {
        T::regs()
            .timctl(self.instance.into())
            .modify(|v| v.set_trgsel(value));
    }

    #[inline]
    pub fn set_trigger_polarity(&mut self, value: bool) {
        T::regs()
            .timctl(self.instance.into())
            .modify(|v| v.set_trgpol(value));
    }

    #[inline]
    pub fn set_pin_config(&mut self, value: u8) {
        T::regs()
            .timctl(self.instance.into())
            .modify(|v| v.set_pincfg(value.into()));
    }

    #[inline]
    pub fn set_pin_polarity(&mut self, value: bool) {
        T::regs()
            .timctl(self.instance as usize)
            .modify(|v| v.set_pinpol(value));
    }

    #[inline]
    pub fn set_mode(&mut self, value: u8) {
        T::regs()
            .timctl(self.instance.into())
            .modify(|v| v.set_timod(value.into()));
    }

    #[inline]
    pub fn set_pin(&mut self, value: u8) {
        T::regs()
            .timctl(self.instance.into())
            .modify(|v| v.set_pinsel(value));
    }

    #[inline]
    pub fn set_decrement(&mut self, value: u8) {
        T::regs()
            .timcfg(self.instance.into())
            .modify(|v| v.set_timdec(value.into()));
    }

    #[inline]
    pub fn set_output(&mut self, value: u8) {
        T::regs()
            .timcfg(self.instance.into())
            .modify(|v| v.set_timout(value.into()));
    }
}

mod sealed {
    use super::*;

    pub trait Instance {
        type Interrupt: interrupt::typelevel::Interrupt;

        fn regs() -> pac::flexio::Flexio;
    }
}

pub trait Instance: sealed::Instance + 'static {}

macro_rules! impl_instance {
    ($inst:ident, $irq:ident) => {
        impl sealed::Instance for peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;

            fn regs() -> pac::flexio::Flexio {
                pac::$inst
            }
        }
        impl Instance for peripherals::$inst {}
    };
}

impl_instance!(FLEXIO1, FLEXIO1);
