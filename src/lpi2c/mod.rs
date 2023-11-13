use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::interrupt::{LPI2C1, LPI2C2};
use crate::{interrupt, pac, peripherals, Peripheral};
use core::future;
use core::marker::PhantomData;
use core::task::Poll;
use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
use pac::lpi2c::vals::*;

#[cfg(feature = "defmt")]
use defmt::println;

/// I2C error abort reason
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AbortReason {
    /// A bus operation was not acknowledged, e.g. due to the addressed device
    /// not being available on the bus or the device not being ready to process
    /// requests at the moment
    NoAcknowledge,
    /// The arbitration was lost, e.g. electrical problems with the clock signal
    ArbitrationLoss,
    /// Transmit ended with data still in fifo
    TxNotEmpty(u16),
    Other(u32),
}

/// I2C error
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// I2C abort with error
    Abort(AbortReason),
    /// User passed in a read buffer that was 0 length
    InvalidReadBufferLength,
    /// User passed in a write buffer that was 0 length
    InvalidWriteBufferLength,
    /// Target i2c address is out of range
    AddressOutOfRange(u16),
    /// Target i2c address is reserved
    AddressReserved(u16),
    /// FIFO is full
    FifoError,
    /// Bus is busy
    Busy,
}

#[non_exhaustive]
#[derive(Copy, Clone, Default)]
pub struct Config {
    pub frequency: u32,
}

pub struct InterruptHandler<T: Instance> {
    _i2c: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    // Mask interrupts and wake any task waiting for this interrupt
    unsafe fn on_interrupt() {
        let msr = T::regs().msr().read();
        T::Interrupt::disable();
        T::waker().wake();
    }
}

pub struct Lpi2c<'d, T: Instance, M: Mode> {
    phantom: PhantomData<(&'d mut T, M)>,

    target_address: u16,
}

impl<'d, T: Instance + 'd> Lpi2c<'d, T, Async> {
    pub fn new_async(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        config: Config,
    ) -> Self {
        let i2c = Self::new_inner(peri, scl, sda, config);

        i2c
    }

    /// Calls `f` to check if we are ready or not.
    /// If not, `g` is called once the waker is set (to eg enable the required interrupts).
    async fn wait_on<F, U, G>(&mut self, mut f: F, mut g: G) -> U
    where
        F: FnMut(&mut Self) -> Poll<U>,
        G: FnMut(&mut Self),
    {
        future::poll_fn(|cx| {
            let r = f(self);

            if r.is_pending() {
                T::waker().register(cx.waker());
                g(self);
            }
            r
        })
        .await
    }

    async fn read_internal(&mut self, buffer: &mut [u8], send_stop: bool) -> Result<(), Error> {
        if buffer.is_empty() {
            return Err(Error::InvalidReadBufferLength);
        }

        let p = T::regs();

        p.mtdr().write(|v| {
            v.set_cmd(Cmd::GENERATE_START_AND_TRANSMIT_ADDRESS_IN_DATA_7_THROUGH_0);
            v.set_data((self.target_address << 1 | 1) as u8);
        });

        let mut abort_reason = Ok(());

        for chunk in buffer.chunks_mut(256) {
            self.clear_msr();

            p.mtdr().write(|v| {
                v.set_cmd(Cmd::RECEIVE_DATA_7_THROUGH_0_PLUS_ONE);
                v.set_data(chunk.len().saturating_sub(1) as u8);
            });

            for mut batch in chunk.chunks_mut(Self::rx_fifo_max_capacity() as usize) {
                while !batch.is_empty() {
                    let water = batch
                        .len()
                        .min(Self::rx_fifo_max_capacity() as _)
                        .saturating_sub(1) as u8;
                    p.mfcr().modify(|r| r.set_rxwater(water));

                    let res = self
                        .wait_on(
                            |me| {
                                let rxfifo = Self::rx_fifo_len();
                                if let Err(abort_reason) = me.check_errors() {
                                    Poll::Ready(Err(abort_reason))
                                } else if rxfifo >= batch.len().div_ceil(2) as u8 {
                                    Poll::Ready(Ok(rxfifo))
                                } else {
                                    Poll::Pending
                                }
                            },
                            |_me| {
                                p.mier().write(|r| {
                                    r.set_rdie(true);
                                    r.set_sdie(true);
                                    r.set_ndie(true);
                                    r.set_alie(true);
                                    r.set_feie(true);
                                });

                                unsafe { T::Interrupt::enable() };
                            },
                        )
                        .await;

                    match res {
                        Err(reason) => {
                            abort_reason = Err(reason);
                            break;
                        }
                        Ok(rxfifo) => {
                            for byte in &mut batch[..rxfifo as usize] {
                                *byte = self.read_data_register().unwrap();
                            }

                            batch = &mut batch[rxfifo as usize..];
                        }
                    };
                }
            }
        }

        self.wait_stop_det(abort_reason, send_stop).await
    }

    async fn write_internal(
        &mut self,
        bytes: impl IntoIterator<Item = u8>,
        send_stop: bool,
    ) -> Result<(), Error> {
        let p = T::regs();

        let mut bytes = bytes.into_iter().peekable();

        if bytes.peek().is_some() {
            let p = T::regs();
            p.mtdr().write(|v| {
                v.set_cmd(Cmd::GENERATE_START_AND_TRANSMIT_ADDRESS_IN_DATA_7_THROUGH_0);
                v.set_data((self.target_address << 1) as u8);
            });
        }

        let res = 'xmit: loop {
            let tx_fifo_space = Self::tx_fifo_capacity();
            self.clear_msr();

            for _ in 0..tx_fifo_space {
                if let Some(byte) = bytes.next() {
                    p.mtdr().write(|v| {
                        v.set_cmd(Cmd::TRANSMIT_DATA_7_THROUGH_0);
                        v.set_data(byte);
                    });
                } else {
                    break 'xmit Ok(());
                }
            }

            let res = self
                .wait_on(
                    |me| {
                        if let abort_reason @ Err(_) = me.check_errors() {
                            Poll::Ready(abort_reason)
                        } else if !Self::tx_fifo_full() {
                            // resume if there's any space free in the tx fifo
                            Poll::Ready(Ok(()))
                        } else {
                            Poll::Pending
                        }
                    },
                    |_me| {
                        p.mier().write(|r| {
                            r.set_tdie(true);
                            r.set_sdie(true);
                            r.set_ndie(true);
                            r.set_alie(true);
                            r.set_epie(true);
                            r.set_feie(true);
                        });

                        T::Interrupt::unpend();
                        unsafe { T::Interrupt::enable() };
                    },
                )
                .await;
            if res.is_err() {
                break res;
            }
        };

        self.wait_stop_det(res, send_stop).await
    }

    /// Helper to wait for a stop bit, for both tx and rx. If we had an abort,
    /// then we'll get a hardware-generated stop, otherwise wait for a stop if
    /// we're expecting it.
    ///
    /// Also handles an abort which arises while processing the tx fifo.
    async fn wait_stop_det(
        &mut self,
        had_abort: Result<(), Error>,
        do_stop: bool,
    ) -> Result<(), Error> {
        if had_abort.is_err() || do_stop {
            let p = T::regs();

            let had_abort2 = self
                .wait_on(
                    |me| {
                        // We could see an abort while processing fifo backlog,
                        // so handle it here.
                        let abort = me.check_errors();
                        if had_abort.is_ok() && abort.is_err() {
                            Poll::Ready(abort)
                        } else if p.msr().read().sdf() {
                            me.clear_msr();

                            Poll::Ready(Ok(()))
                        } else {
                            Poll::Pending
                        }
                    },
                    |_me| {
                        p.mier().write(|r| {
                            r.set_sdie(true);
                            r.set_ndie(true);
                            r.set_alie(true);
                            r.set_epie(true);
                            r.set_feie(true);
                        });

                        T::Interrupt::unpend();
                        unsafe { T::Interrupt::enable() };
                    },
                )
                .await;

            had_abort.and(had_abort2)
        } else {
            had_abort
        }
    }

    pub async fn read(&mut self, addr: u16, buffer: &mut [u8]) -> Result<(), Error> {
        self.setup(addr)?;
        self.clear_fifo();
        self.read_internal(buffer, true).await
    }

    pub async fn write(
        &mut self,
        addr: u16,
        bytes: impl IntoIterator<Item = u8>,
    ) -> Result<(), Error> {
        self.setup(addr)?;
        self.clear_fifo();
        self.write_internal(bytes, true).await
    }

    pub async fn write_read(
        &mut self,
        address: u8,
        bytes: impl IntoIterator<Item = u8>,
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        self.setup(address as _)?;
        self.clear_fifo();
        self.write_internal(bytes, false).await?;
        self.read_internal(buffer, true).await
    }
}

impl<'d, T: Instance + 'd, M: Mode> Lpi2c<'d, T, M> {
    fn new_inner(
        peri: impl Peripheral<P = T> + 'd,
        _scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        _sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        _config: Config,
    ) -> Self {
        into_ref!(peri);

        pac::CCM.ccgr2().modify(|r| {
            r.set_cg3(0b11);
            r.set_cg4(0b11);
        });

        //locator(CCGR6, CG12),
        Self::reset();

        //// I2C disabled due to reset.
        let timings = Timing::ideal(60_000_000, ClockSpeed::KHz100);
        Self::set_timings(&timings);

        T::regs().mfcr().write(|v| {
            v.set_txwater(1);
            v.set_rxwater(1);
        });

        T::regs().mcr().write(|v| v.set_men(true));

        cortex_m::asm::delay(100000);

        Self {
            phantom: PhantomData,
            target_address: 0,
        }
    }

    fn set_timings(timings: &Timing) {
        let clock_config = timings.clock_configuration();
        let p = T::regs();

        p.mccr0().write(|v| {
            v.set_clklo(clock_config.clklo);
            v.set_clkhi(clock_config.clkhi);
            v.set_sethold(clock_config.sethold);
            v.set_datavd(clock_config.datavd);
        });

        p.mcfgr1().write(|v| {
            match timings.prescaler() {
                Prescaler::Prescaler1 => v.set_prescale(Prescale::DIVIDE_BY_1),
                Prescaler::Prescaler2 => v.set_prescale(Prescale::DIVIDE_BY_2),
                Prescaler::Prescaler4 => v.set_prescale(Prescale::DIVIDE_BY_4),
                Prescaler::Prescaler8 => v.set_prescale(Prescale::DIVIDE_BY_8),
                Prescaler::Prescaler16 => v.set_prescale(Prescale::DIVIDE_BY_16),
                Prescaler::Prescaler32 => v.set_prescale(Prescale::DIVIDE_BY_32),
                Prescaler::Prescaler64 => v.set_prescale(Prescale::DIVIDE_BY_64),
                Prescaler::Prescaler128 => v.set_prescale(Prescale::DIVIDE_BY_128),
            };

            v.set_autostop(true);
        });

        p.mcfgr2().write(|v| {
            v.set_filtsda(clock_config.filtsda);
            v.set_filtscl(clock_config.filtscl);
            v.set_busidle(timings.busidle as u16);
        })
    }

    pub fn enabled() -> bool {
        T::regs().mcr().read().men()
    }

    /// Reset the controller.
    ///
    /// Note that this may not not reset all peripheral state, like the controller
    /// enabled state.
    pub fn reset() {
        let mcr = T::regs().mcr();

        mcr.modify(|v| v.set_rst(true));

        while mcr.read().rst() {
            mcr.modify(|v| v.set_rst(false));
        }
    }

    /// Resets the transmit and receive FIFOs.
    #[inline(always)]
    pub fn clear_fifo(&mut self) {
        T::regs().mcr().modify(|v| {
            v.set_rrf(true);
            v.set_rtf(true);
        })
    }

    /// Read the controller receive data register.
    ///
    /// Returns `None` if there is no data in the receive FIFO.
    #[inline]
    pub fn read_data_register(&self) -> Option<u8> {
        let mrdr = T::regs().mrdr().read();

        if mrdr.rxempty() {
            None
        } else {
            Some(mrdr.data())
        }
    }

    /// Block until there's space in the transmit FIFO..
    ///
    /// Return errors if detected.
    fn blocking_wait_for_transmit(&self) -> Result<(), Error> {
        loop {
            self.check_errors()?;

            if T::regs().msr().read().tdf() {
                T::regs().msr().write(|r| r.set_tdf(true));
                return Ok(());
            }
        }
    }

    /// Block until receiving a byte of data.
    ///
    /// Returns errors if detected.
    fn wait_for_data(&self) -> Result<u8, Error> {
        loop {
            self.check_errors()?;

            if let Some(data) = self.read_data_register() {
                return Ok(data);
            }
        }
    }

    fn check_errors(&self) -> Result<(), Error> {
        let msr = T::regs().msr().read();

        if msr.alf() {
            return Err(Error::Abort(AbortReason::ArbitrationLoss));
        }

        if msr.ndf() {
            return Err(Error::Abort(AbortReason::NoAcknowledge));
        }

        if msr.fef() {
            return Err(Error::FifoError);
        }

        Ok(())
    }

    /// Wait for the end of packet, which happens for STOP or repeated
    /// START conditions.
    ///
    /// Returns errors if detected. This will not unblock for a non-repeated
    /// START.
    fn wait_for_end_of_packet(&self) -> Result<(), Error> {
        loop {
            self.check_errors()?;

            if T::regs().msr().read().epf() {
                return Ok(());
            }
        }
    }

    #[inline]
    pub fn rx_watermark(&self) -> u8 {
        T::regs().mfcr().read().rxwater()
    }

    #[inline]
    pub fn tx_watermark(&self) -> u8 {
        T::regs().mfcr().read().txwater()
    }

    #[inline]
    fn tx_fifo_full() -> bool {
        Self::tx_fifo_capacity() == 0
    }

    #[inline]
    fn tx_fifo_capacity() -> u8 {
        let p = T::regs();
        4 - p.mfsr().read().txcount()
    }

    #[inline]
    fn rx_fifo_len() -> u8 {
        let p = T::regs();
        p.mfsr().read().rxcount()
    }

    #[inline]
    const fn rx_fifo_max_capacity() -> u8 {
        4
    }

    fn setup(&mut self, address: u16) -> Result<(), Error> {
        if address >= 0x80 {
            return Err(Error::AddressOutOfRange(address));
        }

        //if i2c_reserved_addr(addr) {
        //    return Err(Error::AddressReserved(addr));
        //}

        self.target_address = address;

        Ok(())
    }

    fn blocking_read_internal(&mut self, buf: &mut [u8], send_stop: bool) -> Result<(), Error> {
        let p = T::regs();

        if buf.is_empty() {
            return Err(Error::InvalidReadBufferLength);
        }

        self.blocking_wait_for_transmit()?;

        p.mtdr().write(|v| {
            v.set_cmd(Cmd::GENERATE_START_AND_TRANSMIT_ADDRESS_IN_DATA_7_THROUGH_0);
            v.set_data((self.target_address << 1 | 1) as _);
        });

        self.blocking_wait_for_transmit()?;

        p.mtdr().write(|v| {
            v.set_cmd(Cmd::RECEIVE_DATA_7_THROUGH_0_PLUS_ONE);
            v.set_data(buf.len().saturating_sub(1) as u8);
        });

        for byte in buf.iter_mut() {
            *byte = self.wait_for_data()?;
        }

        if send_stop {
            self.wait_for_end_of_packet()?;
        }

        Ok(())
    }

    fn blocking_write_internal(&mut self, buf: &[u8], send_stop: bool) -> Result<(), Error> {
        if buf.is_empty() {
            return Err(Error::InvalidWriteBufferLength);
        }

        let p = T::regs();

        self.blocking_wait_for_transmit()?;

        p.mtdr().write(|v| {
            v.set_cmd(Cmd::GENERATE_START_AND_TRANSMIT_ADDRESS_IN_DATA_7_THROUGH_0);
            v.set_data((self.target_address << 1) as u8);
        });

        for byte in buf {
            self.blocking_wait_for_transmit()?;

            p.mtdr().write(|v| {
                v.set_cmd(Cmd::TRANSMIT_DATA_7_THROUGH_0);
                v.set_data(*byte);
            });
        }

        if send_stop {
            self.wait_for_end_of_packet()?;
        }

        Ok(())
    }

    fn blocking_stop(&mut self) -> Result<(), Error> {
        self.blocking_wait_for_transmit()?;

        T::regs().mtdr().write(|v| {
            v.set_cmd(Cmd::GENERATE_STOP_CONDITION);
        });

        Ok(())
    }

    // =========================
    // Blocking public API
    // =========================

    pub fn blocking_read(&mut self, address: u8, buf: &mut [u8]) -> Result<(), Error> {
        self.setup(address.into())?;
        self.clear_fifo();
        self.clear_msr();

        self.blocking_read_internal(buf, true)
    }

    pub fn blocking_write(&mut self, address: u8, buf: &[u8]) -> Result<(), Error> {
        self.setup(address.into())?;
        self.clear_fifo();
        self.clear_msr();

        self.blocking_write_internal(buf, true)
    }

    pub fn blocking_write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Error> {
        self.setup(address.into())?;
        self.clear_fifo();
        self.clear_msr();

        if self.bus_busy() {
            return Err(Error::Busy);
        }

        self.blocking_write_internal(write, false)?;
        self.blocking_read_internal(read, true)?;

        Ok(())
    }

    pub fn bus_busy(&self) -> bool {
        let msr = T::regs().msr().read();

        msr.bbf() && !msr.mbf()
    }

    fn clear_msr(&mut self) {
        let p = T::regs();
        p.msr().modify(|r| *r);
    }
}

pub fn i2c_reserved_addr(addr: u16) -> bool {
    ((addr & 0x78) == 0 || (addr & 0x78) == 0x78) && addr != 0
}

/// Clock configuration fields.
///
/// These fields are written directly to the clock configuration register.
/// All values are written as-is to the register fields. Values that are
/// less than eight bits are truncated by the implementation. You're
/// responsible for making sure that these parameters meet their timing
/// parameter restrictions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfiguration {
    /// Clock high period.
    ///
    /// Minimum number of cycles that the SCL clock is driven high.
    pub clkhi: u8,
    /// Clock low period.
    ///
    /// Minimum number of cycles that the SCL clock is driven low.
    pub clklo: u8,
    /// Setup hold delay.
    ///
    /// Minimum number of cycles that's used for
    /// - START condition hold
    /// - repeated START setup & hold
    /// - START condition setup
    pub sethold: u8,
    /// Data valid delay.
    ///
    /// Minimum number of cycles for SDA data hold. Must be less than
    /// the minimum SCL low period.
    pub datavd: u8,
    /// Glitch filter SDA.
    ///
    /// Only four bits large. Value of zero represents "no filter," and
    /// non-zero values represent filtered cycles.
    pub filtsda: u8,
    /// Glitch filter for SCL.
    ///
    /// Only four bits large. Value of zero represents "no filter," and
    /// non-zero values represent filtered cycles.
    pub filtscl: u8,
}

/// LPI2C timing parameters.
///
/// The implementation computes BUSIDLE based on the clock configuration values,
/// but you can override this after construction.
///
/// The simplest way to construct a `Timing` is to use [`ideal()`](Timing::ideal).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    clock_configuration: ClockConfiguration,
    prescaler: Prescaler,
    busidle: u32,
}

/// Computes SCL and SDA latency cycles.
///
/// See table 47.3 in the reference manual. `risetime` is an estimate of the number
/// of clock cycles for the line to rise. Ideally, this is zero.
const fn line_latency_cycles(filter: u8, risetime: u8, prescaler: Prescaler) -> u8 {
    (2 + filter + risetime) / prescaler.divider()
}

/// Assuming that CLKHI = 2 * CLKLO = CLK, compute CLK. Saturate at u8::MAX.
///
/// BAUD = (HZ / (2 ^ PRESCALER)) / (3CLK + 2 + SCL_LATENCY)
///
/// Solve for CLK:
///
/// CLK = (HZ / (2 ^ PRESCALER) / BAUD - SCL_LATENCY - 2) / 3
///
/// Keep in mind that CLKHI and CLKLO are 6 bit fields, so the saturation value is still
/// out of range.

/// Clock speed.
#[derive(Clone, Copy, Debug)]
pub enum ClockSpeed {
    /// 100 KHz.
    KHz100,
    /// 400 KHz.
    KHz400,
    /// 1 MHz.
    MHz1,
}

impl ClockSpeed {
    const fn frequency(self) -> u32 {
        match self {
            ClockSpeed::KHz100 => 100_000,
            ClockSpeed::KHz400 => 400_000,
            ClockSpeed::MHz1 => 1_000_000,
        }
    }
}

const fn compute_clk(hz: u32, baud: ClockSpeed, prescaler: Prescaler, scl_latency: u8) -> u8 {
    let clk: u32 =
        (hz / prescaler.divider() as u32 / baud.frequency() - scl_latency as u32 - 2) / 3;
    if clk > 0xFF {
        0xFFu8
    } else {
        clk as u8
    }
}

impl Timing {
    /// Compute timing parameters assuming an ideal I2C bus.
    ///
    /// This constructor assumes that
    ///
    /// - the SDA / SCL rise times are negligible (take less than one functional clock cycle).
    /// - there's no need for glitch filters (FLITSCL = FILTSDA = 0).
    ///
    /// These assumptions may not hold true for high clock speeds and I2C bus loadings.
    /// If that's the case, you may find it's better to define timing parameters yourself.
    ///
    /// Note that this function can run at compile time. Consider evaluating in a const
    /// context to avoid the possibility of panics.
    ///
    /// # Panics
    ///
    /// After evaluating all prescalars, this function panics if the computed clock period
    /// cannot be represented in the 6 bits available for the configuration.
    pub fn ideal(clock_hz: u32, clock_speed: ClockSpeed) -> Self {
        const PRESCALERS: [Prescaler; 8] = [
            Prescaler::Prescaler1,
            Prescaler::Prescaler2,
            Prescaler::Prescaler4,
            Prescaler::Prescaler8,
            Prescaler::Prescaler16,
            Prescaler::Prescaler32,
            Prescaler::Prescaler64,
            Prescaler::Prescaler128,
        ];
        /// 6 bits available for all clock configurations.
        const CLOCK_PERIOD_MAX_VAL: u8 = 0x3Fu8;

        // Can't write a for loop in a const function...
        let mut clk = 0xFFu8;
        let mut idx = 0usize;
        while idx < PRESCALERS.len() {
            // Assuming no filters and rise times less than one clock cycle.
            let scl_latency = line_latency_cycles(0, 0, PRESCALERS[idx]);
            clk = compute_clk(clock_hz, clock_speed, PRESCALERS[idx], scl_latency);
            if clk.saturating_mul(2) <= CLOCK_PERIOD_MAX_VAL {
                break;
            }
            idx += 1;
        }

        assert!(
            clk.saturating_mul(2) <= CLOCK_PERIOD_MAX_VAL,
            "Could not compute CLKHI / CLKLO"
        );
        let prescaler = PRESCALERS[idx];

        let mut clkhi = clk;
        if clkhi < 0x01 {
            clkhi = 0x01;
        }

        let mut clklo = clk * 2;
        if clklo < 0x03 {
            clklo = 0x03;
        }

        // No need to assert CLKLO x (2 ^ PRESCALE) > SCL_LATENCY.
        // By SCL_LATENCY expansion,
        //
        //  CLKLO x (2 ^ PRESCALE) > (2 + FILTSCL + SCL_RISETIME) / (2 ^ PRESCALE)
        //
        // We use 0 for FILTSCL and assume a rise time less than 1 cycle (so, 0).
        // The inequality becomes
        //
        //  CLKLO > 2 / (2 ^ (2 * PRESCALE))
        //  CLKLO > 2 if PRESCALE = 0 (Prescaler1)
        //  CLKLO > 0 if PRESCALE > 0 (Prescaler2, Prescaler4, ...)
        //
        // So we're covered by the CLKLO >= 0x03 restriction.

        // Wait at least CLKHI cycles for (repeat) start / stop.
        let mut sethold = clkhi;
        if sethold < 0x02 {
            sethold = 0x02;
        }

        // Assume data valid after CLHI is high for half its cycles.
        let mut datavd = clkhi / 2;
        if datavd < 0x01 {
            datavd = 0x01;
        }

        Self::new(
            ClockConfiguration {
                clkhi,
                clklo,
                sethold,
                datavd,
                filtsda: 0,
                filtscl: 0,
            },
            prescaler,
        )
    }

    /// Computes timing parameters assuming an ideal circuit.
    ///
    ///
    /// Define LPI2C timings by the clock configuration values, and a prescaler.
    pub fn new(clock_configuration: ClockConfiguration, prescaler: Prescaler) -> Self {
        const fn max(left: u32, right: u32) -> u32 {
            if left > right {
                left
            } else {
                right
            }
        }
        let busidle = max(
            (clock_configuration.clklo as u32 + clock_configuration.sethold as u32 + 2) * 2,
            clock_configuration.clkhi as u32 + 1,
        );
        Self {
            clock_configuration,
            prescaler,
            busidle,
        }
    }
    /// Returns the clock configuration.
    pub const fn clock_configuration(&self) -> ClockConfiguration {
        self.clock_configuration
    }
    /// Returns the prescaler.
    pub const fn prescaler(&self) -> Prescaler {
        self.prescaler
    }
    /// Override the BUSIDLE parameter.
    ///
    /// The minimum BUSIDLE is computed by CLKLO, SETHOLD, and CLKHI. Use
    /// this method to override the value.
    pub const fn override_busidle(mut self, busidle: u32) -> Self {
        self.busidle = busidle;
        self
    }
}

/// Source clock prescaler.
///
/// Affects all timing, except for the glitch filters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Prescaler {
    /// Divide the source clock by 1.
    Prescaler1,
    /// Divide the source clock by 2.
    Prescaler2,
    /// Divide the source clock by 4.
    Prescaler4,
    /// Divide the source clock by 8.
    Prescaler8,
    /// Divide the source clock by 16.
    Prescaler16,
    /// Divide the source clock by 32.
    Prescaler32,
    /// Divide the source clock by 64.
    Prescaler64,
    /// Divide the source clock by 128.
    Prescaler128,
}

impl Prescaler {
    /// Returns the divider value for this prescaler.
    ///
    /// `Prescaler8` produces the value '8'.
    pub const fn divider(self) -> u8 {
        1 << self as u8
    }
}

mod sealed {
    use embassy_sync::waitqueue::AtomicWaker;

    use crate::interrupt;

    pub trait Instance {
        type Interrupt: interrupt::typelevel::Interrupt;

        fn regs() -> crate::pac::lpi2c::Lpi2c;
        //fn reset() -> crate::pac::resets::regs::Peripherals;
        fn waker() -> &'static AtomicWaker;
    }

    pub trait Mode {}

    pub trait SdaPin<T: Instance> {}
    pub trait SclPin<T: Instance> {}
}

pub trait Mode: sealed::Mode {}

pub trait Instance: sealed::Instance {}

macro_rules! impl_instance {
    ($type:ident, $irq:ident) => {
        impl sealed::Instance for peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;

            #[inline]
            fn regs() -> pac::lpi2c::Lpi2c {
                pac::$type
            }

            //#[inline]
            //fn reset() -> pac::resets::regs::Peripherals {
            //    let mut ret = pac::resets::regs::Peripherals::default();
            //    ret.$reset(true);
            //    ret
            //}

            #[inline]
            fn waker() -> &'static AtomicWaker {
                static WAKER: AtomicWaker = AtomicWaker::new();

                &WAKER
            }
        }
        impl Instance for peripherals::$type {}
    };
}

impl_instance!(LPI2C1, LPI2C1);
impl_instance!(LPI2C2, LPI2C2);

macro_rules! impl_mode {
    ($name:ident) => {
        impl sealed::Mode for $name {}
        impl Mode for $name {}
    };
}

pub struct Blocking;
pub struct Async;

impl_mode!(Blocking);
impl_mode!(Async);

pub trait SdaPin<T: Instance>: sealed::SdaPin<T> {}
pub trait SclPin<T: Instance>: sealed::SclPin<T> {}

macro_rules! impl_pin {
    ($pin:ident, $instance:ident, $function:ident) => {
        impl sealed::$function<peripherals::$instance> for peripherals::$pin {}
        impl $function<peripherals::$instance> for peripherals::$pin {}
    };
}

impl_pin!(GPIO_01, LPI2C1, SdaPin);
impl_pin!(GPIO_02, LPI2C1, SclPin);
impl_pin!(GPIO_11, LPI2C1, SdaPin);
impl_pin!(GPIO_12, LPI2C1, SclPin);
impl_pin!(GPIO_SD_05, LPI2C1, SdaPin);
impl_pin!(GPIO_SD_06, LPI2C1, SclPin);
impl_pin!(GPIO_AD_13, LPI2C1, SdaPin);
impl_pin!(GPIO_AD_14, LPI2C1, SclPin);

impl_pin!(GPIO_09, LPI2C2, SdaPin);
impl_pin!(GPIO_10, LPI2C2, SclPin);
impl_pin!(GPIO_SD_07, LPI2C2, SdaPin);
impl_pin!(GPIO_SD_08, LPI2C2, SclPin);
impl_pin!(GPIO_AD_07, LPI2C2, SdaPin);
impl_pin!(GPIO_AD_08, LPI2C2, SclPin);
impl_pin!(GPIO_AD_01, LPI2C2, SdaPin);
impl_pin!(GPIO_AD_02, LPI2C2, SclPin);

impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::i2c::Read for Lpi2c<'d, T, M> {
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, buffer)
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::i2c::Write for Lpi2c<'d, T, M> {
    type Error = Error;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, bytes)
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::i2c::WriteRead for Lpi2c<'d, T, M> {
    type Error = Error;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.blocking_write_read(address, bytes, buffer)
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::i2c::Transactional for Lpi2c<'d, T, M> {
    type Error = Error;

    fn exec(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_02::blocking::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.setup(address.into())?;
        for i in 0..operations.len() {
            let last = i == operations.len() - 1;
            match &mut operations[i] {
                embedded_hal_02::blocking::i2c::Operation::Read(buf) => {
                    self.blocking_read_internal(buf, last)?
                }
                embedded_hal_02::blocking::i2c::Operation::Write(buf) => {
                    self.blocking_write_internal(buf, last)?
                }
            }
        }
        Ok(())
    }
}

impl embedded_hal_1::i2c::Error for Error {
    fn kind(&self) -> embedded_hal_1::i2c::ErrorKind {
        match *self {
            Self::Abort(AbortReason::ArbitrationLoss) => {
                embedded_hal_1::i2c::ErrorKind::ArbitrationLoss
            }
            Self::Abort(AbortReason::NoAcknowledge) => {
                embedded_hal_1::i2c::ErrorKind::NoAcknowledge(
                    embedded_hal_1::i2c::NoAcknowledgeSource::Address,
                )
            }
            Self::Abort(AbortReason::TxNotEmpty(_)) => embedded_hal_1::i2c::ErrorKind::Other,
            Self::Abort(AbortReason::Other(_)) => embedded_hal_1::i2c::ErrorKind::Other,
            Self::InvalidReadBufferLength => embedded_hal_1::i2c::ErrorKind::Other,
            Self::InvalidWriteBufferLength => embedded_hal_1::i2c::ErrorKind::Other,
            Self::AddressOutOfRange(_) => embedded_hal_1::i2c::ErrorKind::Other,
            Self::AddressReserved(_) => embedded_hal_1::i2c::ErrorKind::Other,
            Self::FifoError => embedded_hal_1::i2c::ErrorKind::Other,
            Self::Busy => embedded_hal_1::i2c::ErrorKind::Other,
        }
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_1::i2c::ErrorType for Lpi2c<'d, T, M> {
    type Error = Error;
}

impl<'d, A, T> embedded_hal_async::i2c::I2c<A> for Lpi2c<'d, T, Async>
where
    A: embedded_hal_async::i2c::AddressMode + Into<u16> + 'static,
    T: Instance + 'd,
{
    async fn read(&mut self, address: A, read: &mut [u8]) -> Result<(), Self::Error> {
        let addr: u16 = address.into();

        self.setup(addr)?;
        self.clear_msr();
        self.clear_fifo();
        self.read_internal(read, false).await
    }

    async fn write(&mut self, address: A, write: &[u8]) -> Result<(), Self::Error> {
        let addr: u16 = address.into();

        self.setup(addr)?;
        self.clear_msr();
        self.clear_fifo();
        self.write_internal(write.iter().copied(), true).await
    }

    async fn write_read(
        &mut self,
        address: A,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        let addr: u16 = address.into();

        self.setup(addr)?;
        self.clear_msr();
        self.clear_fifo();
        self.write_internal(write.iter().cloned(), false).await?;
        self.read_internal(read, true).await
    }

    async fn transaction(
        &mut self,
        address: A,
        operations: &mut [embedded_hal_1::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        use embedded_hal_1::i2c::Operation;

        let addr: u16 = address.into();

        if !operations.is_empty() {
            self.setup(addr)?;
        }
        let mut iterator = operations.iter_mut();

        while let Some(op) = iterator.next() {
            let last = iterator.len() == 0;

            match op {
                Operation::Read(buffer) => {
                    self.read_internal(buffer, last).await?;
                }
                Operation::Write(buffer) => {
                    self.write_internal(buffer.iter().cloned(), last).await?;
                }
            }
        }
        Ok(())
    }
}
