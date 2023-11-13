use crate::flexspi::nor::generic;
use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::{interrupt, pac, peripherals, Peripheral};
use core::future;
use core::marker::PhantomData;
use core::task::Poll;
use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
use embedded_io_async::Error;

pub mod config;
pub use config::*;

pub mod nor;

pub static LUT_KEY_VAL: u32 = 0x5AF05AF0;

pub enum Port {
    A1,
    A2,
    B1,
    B2,
}

pub const FLASH_BASE: *const u32 = 0x6000_0000 as _;

pub struct Flexspi<'d, T: Instance> {
    _p: &'d PhantomData<T>,
}

impl<'d, T: Instance + 'd> Flexspi<'d, T> {
    pub fn new() -> Self {
        let p = T::regs();

        p.mcr0().modify(|r| {
            r.set_mdis(true);
        });

        let ccm = pac::CCM.ccgr6().read();
        pac::CCM.ccgr6().modify(|r| r.set_cg5(0b11));

        p.mcr0().modify(|r| {
            r.set_mdis(false);
        });

        Self { _p: &PhantomData }
    }

    //fn set_flash_config(&mut self, port: u8, size: u32) {
    //    let p = T::regs();

    //    while !self.bus_idle() {}

    //    p.flsha1cr0().write(|x| x.set_flshsz(size));

    //    // Configure flash parameters
    //    p.flshcr1(port.into()).write(|x| {
    //        //x.set_csinterval()
    //        //x.set_csintervalunit(val)
    //        //x.set_tcsh(val);
    //        //x.set_tcss(val);
    //        //x.set_cas(val);
    //        //x.set_wa(val);
    //    });

    //    p.flshcr2(port.into()).modify(|r| {
    //        r.set_awrwaitunit(pac::flexspi::vals::Awrwaitunit::AWRWAITUNIT_0);
    //        r.set_awrwait(0);
    //        r.set_awrseqnum(0);
    //        r.set_awrseqid(0);
    //        r.set_ardseqnum(0);
    //        r.set_ardseqid(0);
    //        //    configValue |=
    //        //        FLEXSPI_FLSHCR2_AWRWAITUNIT(config->AHBWriteWaitUnit) | FLEXSPI_FLSHCR2_AWRWAIT(config->AHBWriteWaitInterval);
    //        //    if (config->AWRSeqNumber > 0U)
    //        //    {
    //        //        configValue |= FLEXSPI_FLSHCR2_AWRSEQID((uint32_t)config->AWRSeqIndex) |
    //        //                       FLEXSPI_FLSHCR2_AWRSEQNUM((uint32_t)config->AWRSeqNumber - 1U);
    //        //    }
    //        //    //
    //        //    if (config->ARDSeqNumber > 0U)
    //        //    {
    //        //        configValue |= FLEXSPI_FLSHCR2_ARDSEQID((uint32_t)config->ARDSeqIndex) |
    //        //                       FLEXSPI_FLSHCR2_ARDSEQNUM((uint32_t)config->ARDSeqNumber - 1U);
    //        //    }
    //    });

    //    //    /* Configure DLL. */
    //    //    configValue        = FLEXSPI_ConfigureDll(base, config);
    //    //    base->DLLCR[index] = configValue;
    //    //
    //    //    /* Configure write mask. */
    //    //    if (config->enableWriteMask)
    //    //    {
    //    //        base->FLSHCR4 &= ~FLEXSPI_FLSHCR4_WMOPT1_MASK;
    //    //    }
    //    //    else
    //    //    {
    //    //        base->FLSHCR4 |= FLEXSPI_FLSHCR4_WMOPT1_MASK;
    //    //    }
    //    //
    //    //    if (index == 0U) /*PortA*/
    //    //    {
    //    //        base->FLSHCR4 &= ~FLEXSPI_FLSHCR4_WMENA_MASK;
    //    //        base->FLSHCR4 |= FLEXSPI_FLSHCR4_WMENA(config->enableWriteMask);
    //    //    }
    //    //    else
    //    //    {
    //    //        base->FLSHCR4 &= ~FLEXSPI_FLSHCR4_WMENB_MASK;
    //    //        base->FLSHCR4 |= FLEXSPI_FLSHCR4_WMENB(config->enableWriteMask);
    //    //    }
    //    p.mcr0().modify(|r| r.set_mdis(false));

    //    //    /* According to ERR011377, need to delay at least 100 NOPs to ensure the DLL is locked. */
    //    //    statusValue =
    //    //        (index == 0U) ?
    //    //            ((uint32_t)kFLEXSPI_FlashASampleClockSlaveDelayLocked |
    //    //             (uint32_t)kFLEXSPI_FlashASampleClockRefDelayLocked) :
    //    //            ((uint32_t)kFLEXSPI_FlashBSampleClockSlaveDelayLocked | (uint32_t)kFLEXSPI_FlashBSampleClockRefDelayLocked);
    //    //
    //    //    if (0U != (configValue & FLEXSPI_DLLCR_DLLEN_MASK))
    //    //    {
    //    //        /* Wait slave delay line locked and slave reference delay line locked. */
    //    //        while ((base->STS2 & statusValue) != statusValue)
    //    //        {
    //    //        }

    //    cortex_m::asm::delay(100);
    //}

    pub fn set_lut_lock(&mut self, lock: bool) {
        let p = T::regs();

        if lock {
            p.lutkey().write_value(LUT_KEY_VAL);
            p.lutcr().write(|r| r.set_lock(true));
        } else {
            p.lutkey().write_value(LUT_KEY_VAL);
            p.lutcr().write(|r| r.set_unlock(true));
        }
    }

    pub fn update_lut_entry(&mut self, id: u8, seq: Sequence) {
        let p = T::regs();

        while !self.bus_idle() {}

        self.set_lut_lock(false);

        let seq: [u32; 4] = unsafe { core::mem::transmute(seq) };

        for (i, elem) in seq.iter().enumerate() {
            p.lut(4 * (id as usize) + i)
                .write_value(pac::flexspi::regs::Lut(*elem));
        }

        self.set_lut_lock(true);
    }

    pub fn reset(&mut self) {
        T::regs().mcr0().modify(|r| r.set_swreset(true));

        while T::regs().mcr0().read().swreset() {}
    }

    pub fn enable(&mut self) {
        T::regs().mcr0().modify(|r| r.set_mdis(false));
    }

    pub fn disable(&mut self) {
        T::regs().mcr0().modify(|r| r.set_mdis(true));
    }

    pub fn reset_fifos(&mut self, tx: bool, rx: bool) {
        let p = T::regs();

        if tx {
            p.iptxfcr().modify(|r| r.set_clriptxf(true));
        }

        if rx {
            p.iprxfcr().modify(|r| r.set_clriprxf(true));
        }
    }

    #[inline]
    pub fn bus_idle(&self) -> bool {
        let sts = T::regs().sts0().read();

        sts.arbidle() && sts.seqidle()
    }

    #[inline]
    pub fn blocking_wait_bus_idle(&self) {
        while !self.bus_idle() {}
    }

    pub fn set_parallel_command_mode(&mut self, enable: bool) {
        let reg = T::regs().ipcr1();

        reg.modify(|r| r.set_iparen(enable));
    }

    pub fn set_ahb_parallel_mode(&mut self, enable: bool) {
        let reg = T::regs().ahbcr();

        reg.modify(|r| r.set_aparen(enable));
    }
}

mod sealed {
    use embassy_sync::waitqueue::AtomicWaker;

    use crate::interrupt;

    pub trait Instance {
        type Interrupt: interrupt::typelevel::Interrupt;

        fn regs() -> crate::pac::flexspi::Flexspi;
        //fn reset() -> crate::pac::resets::regs::Peripherals;
        fn waker() -> &'static AtomicWaker;
    }

    pub trait Mode {}
}

pub trait Mode: sealed::Mode {}

pub trait Instance: sealed::Instance {}

macro_rules! impl_instance {
    ($type:ident, $irq:ident) => {
        impl sealed::Instance for peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;

            #[inline]
            fn regs() -> pac::flexspi::Flexspi {
                pac::$type
            }

            #[inline]
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();
                &WAKER
            }
        }
        impl Instance for peripherals::$type {}
    };
}

impl_instance!(FLEXSPI, FLEXSPI);
