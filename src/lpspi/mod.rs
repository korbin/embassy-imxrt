use crate::dma::AnyChannel;
use crate::dma::Channel;
use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::interrupt::{LPSPI1, LPSPI2};
use crate::{interrupt, pac, peripherals, Peripheral};
use core::future;
use core::marker::PhantomData;
use core::task::Poll;
use embassy_hal_internal::{into_ref, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
pub use embedded_hal_02::spi::{Phase, Polarity};
use pac::lpspi::vals::*;

/// Possible errors when interfacing the LPSPI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The transaction frame size is incorrect.
    ///
    /// The frame size, in bits, must be between 8 bits and
    /// 4095 bits.
    FrameSize,
    /// FIFO error in the given direction.
    Fifo(Direction),
    /// Bus is busy at the start of a transfer.
    Busy,
    /// Caller provided no data.
    NoData,
}

/// Data direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Transmit direction (leaving the peripheral).
    Tx,
    /// Receive direction (entering the peripheral).
    Rx,
}

/// Bit order.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum BitOrder {
    /// Data is transferred most significant bit first (default).
    #[default]
    Msb,
    /// Data is transferred least significant bit first.
    Lsb,
}

/// Receive sample point behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SamplePoint {
    /// Input data is sampled on SCK edge.
    Edge,
    /// Input data is sampled on delayed SCK edge.
    DelayedEdge,
}

pub struct Lpspi<'d, T: Instance, M: Mode> {
    inner: PeripheralRef<'d, T>,
    phantom: PhantomData<(&'d mut T, M)>,

    tx_dma: Option<PeripheralRef<'d, AnyChannel>>,
    rx_dma: Option<PeripheralRef<'d, AnyChannel>>,
}

impl<'d, T: Instance, M: Mode> Lpspi<'d, T, M> {
    fn new_inner(
        inner: impl Peripheral<P = T> + 'd,
        clk: Option<PeripheralRef<'d, impl ClkPin<T> + 'd>>,
        mosi: Option<PeripheralRef<'d, impl MosiPin<T> + 'd>>,
        miso: Option<PeripheralRef<'d, impl MisoPin<T> + 'd>>,
        cs: Option<PeripheralRef<'d, impl CsPin<T> + 'd>>,
        tx_dma: Option<PeripheralRef<'d, AnyChannel>>,
        rx_dma: Option<PeripheralRef<'d, AnyChannel>>,
    ) -> Self {
        into_ref!(inner);

        //    let p = inner.regs();
        //    let (presc, postdiv) = calc_prescs(config.frequency);

        //    p.cpsr().write(|w| w.set_cpsdvsr(presc));
        //    p.cr0().write(|w| {
        //        w.set_dss(0b0111); // 8bit
        //        w.set_spo(config.polarity == Polarity::IdleHigh);
        //        w.set_sph(config.phase == Phase::CaptureOnSecondTransition);
        //        w.set_scr(postdiv);
        //    });

        //    // Always enable DREQ signals -- harmless if DMA is not listening
        //    p.dmacr().write(|reg| {
        //        reg.set_rxdmae(true);
        //        reg.set_txdmae(true);
        //    });

        //    // finally, enable.
        //    p.cr1().write(|w| w.set_sse(true));

        //    if let Some(pin) = &clk {
        //        pin.gpio().ctrl().write(|w| w.set_funcsel(1));
        //    }
        //    if let Some(pin) = &mosi {
        //        pin.gpio().ctrl().write(|w| w.set_funcsel(1));
        //    }
        //    if let Some(pin) = &miso {
        //        pin.gpio().ctrl().write(|w| w.set_funcsel(1));
        //    }
        //    if let Some(pin) = &cs {
        //        pin.gpio().ctrl().write(|w| w.set_funcsel(1));
        //    }

        Self {
            inner,
            tx_dma,
            rx_dma,
            phantom: PhantomData,
        }
    }

    /// Write data to SPI blocking execution until done.
    pub fn blocking_write(&mut self, data: &[u8]) -> Result<(), Error> {
        todo!();
        //let p = self.inner.regs();
        //for &b in data {
        //    while !p.sr().read().tnf() {}
        //    p.dr().write(|w| w.set_data(b as _));
        //    while !p.sr().read().rne() {}
        //    let _ = p.dr().read();
        //}
        //self.flush()?;
        //Ok(())
    }

    /// Transfer data in place to SPI blocking execution until done.
    pub fn blocking_transfer_in_place(&mut self, data: &mut [u8]) -> Result<(), Error> {
        todo!();
        //let p = self.inner.regs();
        //for b in data {
        //    while !p.sr().read().tnf() {}
        //    p.dr().write(|w| w.set_data(*b as _));
        //    while !p.sr().read().rne() {}
        //    *b = p.dr().read().data() as u8;
        //}
        //self.flush()?;
        //Ok(())
    }

    /// Read data from SPI blocking execution until done.
    pub fn blocking_read(&mut self, data: &mut [u8]) -> Result<(), Error> {
        todo!();
        //let p = self.inner.regs();
        //for b in data {
        //    while !p.sr().read().tnf() {}
        //    p.dr().write(|w| w.set_data(0));
        //    while !p.sr().read().rne() {}
        //    *b = p.dr().read().data() as u8;
        //}
        //self.flush()?;
        //Ok(())
    }

    /// Transfer data to SPI blocking execution until done.
    pub fn blocking_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        todo!();
        //let p = self.inner.regs();
        //let len = read.len().max(write.len());
        //for i in 0..len {
        //    let wb = write.get(i).copied().unwrap_or(0);
        //    while !p.sr().read().tnf() {}
        //    p.dr().write(|w| w.set_data(wb as _));
        //    while !p.sr().read().rne() {}
        //    let rb = p.dr().read().data() as u8;
        //    if let Some(r) = read.get_mut(i) {
        //        *r = rb;
        //    }
        //}
        //self.flush()?;
        //Ok(())
    }

    /// Block execution until SPI is done.
    pub fn flush(&mut self) -> Result<(), Error> {
        todo!();
        //let p = self.inner.regs();
        //while p.sr().read().bsy() {}
        //Ok(())
    }

    /// Set SPI frequency.
    pub fn set_frequency(&mut self, freq: u32) {
        todo!();
        //let (presc, postdiv) = calc_prescs(freq);
        //let p = self.inner.regs();
        //// disable
        //p.cr1().write(|w| w.set_sse(false));

        //// change stuff
        //p.cpsr().write(|w| w.set_cpsdvsr(presc));
        //p.cr0().modify(|w| {
        //    w.set_scr(postdiv);
        //});

        //// enable
        //p.cr1().write(|w| w.set_sse(true));
    }

    /// Indicates if the driver is (`true`) or is not (`false`) enabled.
    pub fn is_enabled(&self) -> bool {
        T::regs().cr().read().men()
    }

    /// Enable (`true`) or disable (`false`) the peripheral.
    pub fn set_enable(&mut self, enable: bool) {
        T::regs().cr().modify(|r| r.set_men(enable));
    }

    /// Reset the driver.
    ///
    /// Note that this may not not reset all peripheral state, like the
    /// enabled state.
    pub fn reset(&mut self) {
        let cr = T::regs().cr();
        cr.modify(|r| r.set_rst(true));
        while cr.read().rst() {
            cr.modify(|r| r.set_rst(false));
        }
    }
    /// Clear any existing data in the SPI receive or transfer FIFOs.
    #[inline]
    pub fn clear_fifo(&mut self, direction: Direction) {
        let cr = T::regs().cr();
        match direction {
            Direction::Tx => cr.modify(|r| r.set_rtf(true)),
            Direction::Rx => cr.modify(|r| r.set_rrf(true)),
        }
    }

    /// Clear both FIFOs.
    pub fn clear_fifos(&mut self) {
        T::regs().cr().modify(|r| {
            r.set_rtf(true);
            r.set_rrf(true);
        });
    }

    /// Place `word` into the transmit FIFO.
    ///
    /// This will result in the value being sent from the LPSPI.
    /// You're responsible for making sure that the transmit FIFO can
    /// fit this word.
    pub fn enqueue_data(&self, word: u32) {
        T::regs().tdr().write_value(word);
    }

    /// Simply read whatever is in the receiver data register.
    fn read_data_unchecked(&self) -> u32 {
        T::regs().rdr().read()
    }

    /// Read the data register.
    ///
    /// Returns `None` if the receive FIFO is empty. Otherwise, returns the complete
    /// read of the register. You're reponsible for interpreting the raw value as
    /// a data word, depending on the frame size.
    pub fn read_data(&mut self) -> Option<u32> {
        if T::regs().rsr().read().rxempty() {
            return None;
        }

        Some(self.read_data_unchecked())
    }

    /// Let the peripheral act as a DMA source.
    ///
    /// After this call, the peripheral will signal to the DMA engine whenever
    /// it has data available to read.
    pub fn enable_dma_receive(&mut self) {
        T::regs().fcr().modify(|r| r.set_rxwater(0));
        T::regs().der().modify(|r| r.set_rdde(true));
    }

    /// Stop the peripheral from acting as a DMA source.
    ///
    /// See the DMA chapter in the reference manual to understand when this
    /// should be called in the DMA transfer lifecycle.
    pub fn disable_dma_receive(&mut self) {
        while T::regs().der().read().rdde() {
            T::regs().der().modify(|r| r.set_rdde(false));
        }
    }

    /// Let the peripheral act as a DMA destination.
    ///
    /// After this call, the peripheral will signal to the DMA engine whenever
    /// it has free space in its transfer buffer.
    pub fn enable_dma_transmit(&mut self) {
        T::regs().fcr().modify(|r| r.set_txwater(0));
        T::regs().der().modify(|r| r.set_tdde(true));
    }

    /// Stop the peripheral from acting as a DMA destination.
    ///
    /// See the DMA chapter in the reference manual to understand when this
    /// should be called in the DMA transfer lifecycle.
    pub fn disable_dma_transmit(&mut self) {
        while T::regs().der().read().tdde() {
            T::regs().der().modify(|r| r.set_tdde(false));
        }
    }
}

impl<'d, T: Instance> Lpspi<'d, T, Async> {
    /// Create an SPI driver in async mode supporting DMA operations.
    pub fn new(
        inner: impl Peripheral<P = T> + 'd,
        clk: impl Peripheral<P = impl ClkPin<T> + 'd> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T> + 'd> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T> + 'd> + 'd,
        tx_dma: impl Peripheral<P = impl Channel> + 'd,
        rx_dma: impl Peripheral<P = impl Channel> + 'd,
        //config: Config,
    ) -> Self {
        into_ref!(tx_dma, rx_dma, clk, mosi, miso);
        Self::new_inner(
            inner,
            Some(clk.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            None,
            Some(tx_dma.map_into()),
            Some(rx_dma.map_into()),
            //config,
        )
    }

    /// Create an SPI driver in async mode supporting DMA write operations only.
    pub fn new_txonly(
        inner: impl Peripheral<P = T> + 'd,
        clk: impl Peripheral<P = impl ClkPin<T> + 'd> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T> + 'd> + 'd,
        tx_dma: impl Peripheral<P = impl Channel> + 'd,
        //config: Config,
    ) -> Self {
        into_ref!(tx_dma, clk, mosi);
        Self::new_inner(
            inner,
            Some(clk.map_into()),
            Some(mosi.map_into()),
            None,
            None,
            Some(tx_dma.map_into()),
            None,
            //config,
        )
    }

    /// Create an SPI driver in async mode supporting DMA read operations only.
    pub fn new_rxonly(
        inner: impl Peripheral<P = T> + 'd,
        clk: impl Peripheral<P = impl ClkPin<T> + 'd> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T> + 'd> + 'd,
        rx_dma: impl Peripheral<P = impl Channel> + 'd,
        //config: Config,
    ) -> Self {
        into_ref!(rx_dma, clk, miso);
        Self::new_inner(
            inner,
            Some(clk.map_into()),
            None,
            Some(miso.map_into()),
            None,
            None,
            Some(rx_dma.map_into()),
            //config,
        )
    }

    /// Write data to SPI using DMA.
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        todo!();
        //let tx_ch = self.tx_dma.as_mut().unwrap();
        //let tx_transfer = unsafe {
        //    // If we don't assign future to a variable, the data register pointer
        //    // is held across an await and makes the future non-Send.
        //    crate::dma::write(tx_ch, buffer, self.inner.regs().dr().as_ptr() as *mut _, T::TX_DREQ)
        //};
        //tx_transfer.await;

        //let p = self.inner.regs();
        //while p.sr().read().bsy() {}

        //// clear RX FIFO contents to prevent stale reads
        //while p.sr().read().rne() {
        //    let _: u16 = p.dr().read().data();
        //}
        //// clear RX overrun interrupt
        //p.icr().write(|w| w.set_roric(true));

        //Ok(())
    }

    /// Read data from SPI using DMA.
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        todo!();
        //// Start RX first. Transfer starts when TX starts, if RX
        //// is not started yet we might lose bytes.
        //let rx_ch = self.rx_dma.as_mut().unwrap();
        //let rx_transfer = unsafe {
        //    // If we don't assign future to a variable, the data register pointer
        //    // is held across an await and makes the future non-Send.
        //    crate::dma::read(
        //        rx_ch,
        //        self.inner.regs().dr().as_ptr() as *const _,
        //        buffer,
        //        T::RX_DREQ,
        //    )
        //};

        //let tx_ch = self.tx_dma.as_mut().unwrap();
        //let tx_transfer = unsafe {
        //    // If we don't assign future to a variable, the data register pointer
        //    // is held across an await and makes the future non-Send.
        //    crate::dma::write_repeated(
        //        tx_ch,
        //        self.inner.regs().dr().as_ptr() as *mut u8,
        //        buffer.len(),
        //        T::TX_DREQ,
        //    )
        //};
        //join(tx_transfer, rx_transfer).await;
        //Ok(())
    }

    /// Transfer data to SPI using DMA.
    pub async fn transfer(&mut self, rx_buffer: &mut [u8], tx_buffer: &[u8]) -> Result<(), Error> {
        self.transfer_inner(rx_buffer, tx_buffer).await
    }

    /// Transfer data in place to SPI using DMA.
    pub async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Error> {
        self.transfer_inner(words, words).await
    }

    async fn transfer_inner(
        &mut self,
        rx_ptr: *mut [u8],
        tx_ptr: *const [u8],
    ) -> Result<(), Error> {
        todo!();
        //let (_, tx_len) = crate::dma::slice_ptr_parts(tx_ptr);
        //let (_, rx_len) = crate::dma::slice_ptr_parts_mut(rx_ptr);

        //// Start RX first. Transfer starts when TX starts, if RX
        //// is not started yet we might lose bytes.
        //let rx_ch = self.rx_dma.as_mut().unwrap();
        //let rx_transfer = unsafe {
        //    // If we don't assign future to a variable, the data register pointer
        //    // is held across an await and makes the future non-Send.
        //    crate::dma::read(
        //        rx_ch,
        //        self.inner.regs().dr().as_ptr() as *const _,
        //        rx_ptr,
        //        T::RX_DREQ,
        //    )
        //};

        //let mut tx_ch = self.tx_dma.as_mut().unwrap();
        //// If we don't assign future to a variable, the data register pointer
        //// is held across an await and makes the future non-Send.
        //let tx_transfer = async {
        //    let p = self.inner.regs();
        //    unsafe {
        //        crate::dma::write(&mut tx_ch, tx_ptr, p.dr().as_ptr() as *mut _, T::TX_DREQ).await;

        //        if rx_len > tx_len {
        //            let write_bytes_len = rx_len - tx_len;
        //            // write dummy data
        //            // this will disable incrementation of the buffers
        //            crate::dma::write_repeated(
        //                tx_ch,
        //                p.dr().as_ptr() as *mut u8,
        //                write_bytes_len,
        //                T::TX_DREQ,
        //            )
        //            .await
        //        }
        //    }
        //};
        //join(tx_transfer, rx_transfer).await;

        //// if tx > rx we should clear any overflow of the FIFO SPI buffer
        //if tx_len > rx_len {
        //    let p = self.inner.regs();
        //    while p.sr().read().bsy() {}

        //    // clear RX FIFO contents to prevent stale reads
        //    while p.sr().read().rne() {
        //        let _: u16 = p.dr().read().data();
        //    }
        //    // clear RX overrun interrupt
        //    p.icr().write(|w| w.set_roric(true));
        //}

        //Ok(())
    }
}

impl<'d, T: Instance> Lpspi<'d, T, Blocking> {
    /// Create an SPI driver in blocking mode.
    pub fn new_blocking(
        inner: impl Peripheral<P = T> + 'd,
        clk: impl Peripheral<P = impl ClkPin<T> + 'd> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T> + 'd> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T> + 'd> + 'd,
        //config: Config,
    ) -> Self {
        into_ref!(clk, mosi, miso);
        Self::new_inner(
            inner,
            Some(clk),
            Some(mosi),
            Some(miso),
            None,
            None,
            None,
            //config,
        )
    }

    /// Create an SPI driver in blocking mode supporting writes only.
    pub fn new_blocking_txonly(
        inner: impl Peripheral<P = T> + 'd,
        clk: impl Peripheral<P = impl ClkPin<T> + 'd> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T> + 'd> + 'd,
        //config: Config,
    ) -> Self {
        into_ref!(clk, mosi);
        Self::new_inner(
            inner,
            Some(clk),
            Some(mosi),
            None,
            None,
            None,
            None,
            //config,
        )
    }

    /// Create an SPI driver in blocking mode supporting reads only.
    pub fn new_blocking_rxonly(
        inner: impl Peripheral<P = T> + 'd,
        clk: impl Peripheral<P = impl ClkPin<T> + 'd> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T> + 'd> + 'd,
        //config: Config,
    ) -> Self {
        into_ref!(clk, miso);

        Self::new_inner(
            inner,
            Some(clk),
            None,
            Some(miso),
            None, //config,
            None,
            None,
        )
    }
}

mod sealed {
    use super::*;

    pub trait Mode {}

    pub trait Instance {
        const TX_DREQ: u8;
        const RX_DREQ: u8;

        fn regs() -> pac::lpspi::Lpspi;
    }
}

/// Mode.
pub trait Mode: sealed::Mode {}

/// SPI instance trait.
pub trait Instance: sealed::Instance {}

macro_rules! impl_instance {
    ($type:ident, $irq:ident, $tx_dreq:expr, $rx_dreq:expr) => {
        impl sealed::Instance for peripherals::$type {
            const TX_DREQ: u8 = $tx_dreq;
            const RX_DREQ: u8 = $rx_dreq;

            fn regs() -> pac::spi::Spi {
                pac::$type
            }
        }
        impl Instance for peripherals::$type {}
    };
}

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

/// CLK pin.
pub trait ClkPin<T: Instance> {}
/// CS pin.
pub trait CsPin<T: Instance> {}
/// MOSI pin.
pub trait MosiPin<T: Instance> {}
/// MISO pin.
pub trait MisoPin<T: Instance> {}

macro_rules! impl_pin {
    ($pin:ident, $instance:ident, $function:ident) => {
        impl $function<peripherals::$instance> for peripherals::$pin {}
    };
}

//impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::spi::Transfer<u8> for Lpspi<'d, T, M> {
//    type Error = Error;
//    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
//        self.blocking_transfer_in_place(words)?;
//        Ok(words)
//    }
//}
//
//impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::spi::Write<u8> for Lpspi<'d, T, M> {
//    type Error = Error;
//
//    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
//        self.blocking_write(words)
//    }
//}
//
//impl embedded_hal_1::spi::Error for Error {
//    fn kind(&self) -> embedded_hal_1::spi::ErrorKind {
//        match *self {}
//    }
//}
//
//impl<'d, T: Instance, M: Mode> embedded_hal_1::spi::ErrorType for Lpspi<'d, T, M> {
//    type Error = Error;
//}
//
//impl<'d, T: Instance, M: Mode> embedded_hal_1::spi::SpiBus<u8> for Lpspi<'d, T, M> {
//    fn flush(&mut self) -> Result<(), Self::Error> {
//        Ok(())
//    }
//
//    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
//        self.blocking_transfer(words, &[])
//    }
//
//    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
//        self.blocking_write(words)
//    }
//
//    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
//        self.blocking_transfer(read, write)
//    }
//
//    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
//        self.blocking_transfer_in_place(words)
//    }
//}
//
//impl<'d, T: Instance> embedded_hal_async::spi::SpiBus<u8> for Lpspi<'d, T, Async> {
//    async fn flush(&mut self) -> Result<(), Self::Error> {
//        Ok(())
//    }
//
//    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
//        self.write(words).await
//    }
//
//    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
//        self.read(words).await
//    }
//
//    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
//        self.transfer(read, write).await
//    }
//
//    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
//        self.transfer_in_place(words).await
//    }
//}
//
//impl<'d, T: Instance, M: Mode> SetConfig for Lpspi<'d, T, M> {
//    type Config = Config;
//    type ConfigError = ();
//    fn set_config(&mut self, config: &Self::Config) -> Result<(), ()> {
//        let p = self.inner.regs();
//        let (presc, postdiv) = calc_prescs(config.frequency);
//        p.cpsr().write(|w| w.set_cpsdvsr(presc));
//        p.cr0().write(|w| {
//            w.set_dss(0b0111); // 8bit
//            w.set_spo(config.polarity == Polarity::IdleHigh);
//            w.set_sph(config.phase == Phase::CaptureOnSecondTransition);
//            w.set_scr(postdiv);
//        });
//
//        Ok(())
//    }
//}
