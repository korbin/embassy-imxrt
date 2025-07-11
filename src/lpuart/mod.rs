use crate::dma::{self, AnyChannel, Channel, Destination, Source};
use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::{interrupt, pac, peripherals, Peri, PeripheralType};
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;
use embassy_futures::{select::select, select::Either};
// Peri is already imported from crate
use pac::ccm::vals::UartClkPodf;
use pac::lpuart::regs::{Baud, Data};
use pac::lpuart::vals::{Idlecfg, Rxfifosize, Txfifosize};

#[cfg(feature = "defmt")]
use defmt::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1 = 0,
    #[doc = "2 stop bits"]
    STOP2 = 1,
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    Overrun,
    Break,
    Parity,
    Framing,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
    pub oversampling: u8,
    /// Invert the tx pin output
    pub invert_tx: bool,
    /// Invert the rx pin input
    pub invert_rx: bool,
    // Invert the rts pin
    pub invert_rts: bool,
    // Invert the cts pin
    pub invert_cts: bool,
}

pub struct InterruptHandler<T: Instance> {
    _uart: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let lpuart = T::regs();

        let stat = lpuart.stat().read();
        let fifo = lpuart.fifo().read();

        let tc = stat.tc();
        let rdrf = stat.rdrf();
        let tdre = stat.tdre();
        let idle = stat.idle();
        let fe = stat.fe();
        let pf = stat.pf();
        let or = stat.or();
        let nf = stat.nf();

        let rxempt = fifo.rxempt();
        let txempt = fifo.txempt();
        let txof = fifo.txof();
        let rxuf = fifo.rxuf();

        if idle {
            T::state().idle_irq.store(true, Ordering::Relaxed);
            T::state().rx_waker.wake();
        }

        lpuart.ctrl().modify(|x| {
            if tdre {
                T::state().tx_waker.wake();
                x.set_tie(false);
            }

            if rdrf {
                T::state().rx_waker.wake();
                x.set_rie(false);
            }
        });

        lpuart.stat().modify(|x| {
            x.set_tc(tc);
            x.set_idle(idle);
            x.set_fe(fe);
            x.set_pf(pf);
            x.set_or(or);
        });

        let water = lpuart.water().read();
    }
}

pub struct LpuartTx<'d, T: Instance> {
    tx_dma: Option<Peri<'d, AnyChannel>>,
    _phantom: &'d PhantomData<T>,
}

impl<'d, T: Instance> LpuartTx<'d, T> {
    /// Indicates if the transmit / receive functions are
    /// (`true`) or are not (`false`) enabled.
    pub fn is_enabled(&self) -> bool {
        let lpuart = T::regs();
        lpuart.ctrl().read().te()
    }

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        for byte in buffer {
            while !T::regs().stat().read().tdre() {}

            T::regs().data().write_value(Data(*byte as u32));
        }

        Ok(())
    }

    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        if self.tx_dma.is_some() {
            self.write_dma(buffer).await?;
        } else {
            self.write_async(buffer).await?;
        }

        Ok(())
    }

    async fn write_async(&mut self, buffer: &[u8]) -> Result<(), Error> {
        for byte in buffer {
            poll_fn(|cx| {
                T::state().tx_waker.register(cx.waker());

                if !T::regs().stat().read().tdre() {
                    T::regs().ctrl().modify(|r| r.set_tie(true));
                    Poll::Pending
                } else {
                    T::regs().stat().modify(|x| {
                        x.set_tdre(true);
                    });
                    Poll::Ready(())
                }
            })
            .await;

            T::regs().data().write_value(Data(*byte as u32));

            embassy_futures::yield_now().await;
        }
        Ok(())
    }

    async fn write_dma(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let tx_dma = unsafe { self.tx_dma.as_mut().unwrap().clone_unchecked() };
        unsafe { crate::dma::write(tx_dma, buffer, self).await };

        Ok(())
    }

    pub fn set_enable(&mut self, enable: bool) {
        let lpuart = T::regs();
        lpuart.ctrl().modify(|x| x.set_te(enable));
    }
    /// Indicates if the bits are inverted.
    #[inline]
    pub fn is_inverted(&self) -> bool {
        let lpuart = T::regs();
        lpuart.ctrl().read().txinv()
    }

    /// Indicates if the FIFO is enabled.
    #[inline]
    pub fn is_fifo_enabled(&self) -> bool {
        let lpuart = T::regs();
        lpuart.fifo().read().txfe()
    }

    /// Returns the FIFO watermark value.
    #[inline]
    pub fn fifo_watermark(&self) -> u32 {
        let lpuart = T::regs();
        lpuart.water().read().txwater().into()
    }

    /// Returns the FIFO watermark value.
    #[inline]
    pub fn fifo_flush(&self) {
        let lpuart = T::regs();
        lpuart.fifo().modify(|x| x.set_txflush(true))
    }

    /// Let the peripheral act as a DMA destination.
    ///
    /// After this call, the peripheral will signal to the DMA engine whenever
    /// it has free space in its transfer buffer.
    pub fn enable_dma(&mut self) {
        let lpuart = T::regs();
        lpuart.baud().modify(|b| b.set_tdmae(true))
    }

    /// Stop the peripheral from acting as a DMA destination.
    ///
    /// See the DMA chapter in the reference manual to understand when this
    /// should be called in the DMA transfer lifecycle.
    pub fn disable_dma(&mut self) {
        let lpuart = T::regs();
        while lpuart.baud().read().tdmae() {
            lpuart.baud().modify(|b| b.set_tdmae(false));
        }
    }
}

impl<'d, W: dma::Word, T: Instance> dma::Destination<W> for LpuartTx<'d, T> {
    fn destination_signal(&self) -> u8 {
        T::TX_DREQ
    }

    fn disable_destination(&mut self) {
        self.disable_dma();
    }

    fn enable_destination(&mut self) {
        self.enable_dma();
    }
    fn destination_address(&self) -> *const W {
        let lpuart = T::regs();
        lpuart.data().as_ptr() as _
    }
    fn destination_type(&self) -> dma::DmaType {
        dma::DmaType::Hardware
    }
}

pub struct LpuartRx<'d, T: Instance> {
    rx_dma: Option<Peri<'d, AnyChannel>>,
    _phantom: &'d PhantomData<T>,
}

impl<'d, T: Instance> LpuartRx<'d, T> {
    /// Indicates if the transmit / receive functions are
    /// (`true`) or are not (`false`) enabled.
    pub fn is_enabled(&self) -> bool {
        let lpuart = T::regs();
        lpuart.ctrl().read().re()
    }

    /// Enable (`true`) or disable (`false`) the transmit / receive
    /// functions.
    pub fn set_enable(&mut self, enable: bool) {
        let lpuart = T::regs();
        lpuart.ctrl().modify(|x| x.set_re(enable));
    }
    pub async fn read(&mut self, mut buffer: &mut [u8]) -> Result<usize, Error> {
        let mut num_bytes: usize = 0;
        let flags = T::regs().stat().read();
        if flags.or() {
            T::regs().stat().modify(|v| v.set_or(true));
        }

        loop {
            let read = T::regs().data().read();

            if read.rxempt() {
                break;
            }

            num_bytes += 1;

            buffer[0] = read.0 as u8;
            buffer = &mut buffer[1..];
        }

        if self.rx_dma.is_some() {
            Ok(num_bytes + self.read_dma(buffer).await?)
        } else {
            Ok(num_bytes + self.read_async(buffer).await?)
        }
    }

    pub async fn read_dma(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        let rx_dma = unsafe { self.rx_dma.as_mut().unwrap().clone_unchecked() };

        T::regs().ctrl().modify(|r| {
            r.set_ilie(true);
        });

        let idle = poll_fn(|cx| {
            T::state().rx_waker.register(cx.waker());

            if T::state().idle_irq.load(Ordering::Acquire) {
                T::state().idle_irq.store(false, Ordering::Relaxed);
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        });

        let init_daddr = Destination::destination_address(buffer) as usize;
        let dma = unsafe { crate::dma::read(rx_dma, self, buffer) };

        let rx_dma = unsafe { self.rx_dma.as_mut().unwrap().clone_unchecked() };

        let res = select(dma, idle).await;

        let final_daddr = rx_dma.previous_destination_address() as usize;
        let addr_delta = final_daddr - init_daddr;

        match res {
            Either::First(_) => {
                //println!("DMA TRANSFER COMPLETED");
            }

            Either::Second(_) => {
                //println!("LPUART WENT IDLE");
            }
        };

        Ok(addr_delta)
    }

    pub async fn read_async(&mut self, mut buffer: &mut [u8]) -> Result<usize, Error> {
        let mut num_bytes: usize = 0;
        while !buffer.is_empty() {
            let byte = poll_fn(|cx| {
                let read = T::regs().data().read();

                T::state().rx_waker.register(cx.waker());

                if read.rxempt() {
                    if T::state().idle_irq.load(Ordering::Acquire) {
                        T::state().idle_irq.store(false, Ordering::Relaxed);
                        return Poll::Ready(None);
                    }

                    T::regs().ctrl().modify(|r| {
                        r.set_rie(true);
                        r.set_ilie(true);
                    });

                    Poll::Pending
                } else {
                    T::regs().stat().modify(|x| {
                        x.set_rdrf(true);
                    });
                    num_bytes += 1;
                    Poll::Ready(Some(read.0 as u8))
                }
            })
            .await;

            if let Some(byte) = byte {
                buffer[0] = byte;

                buffer = &mut buffer[1..];
            } else {
                break;
            }
        }

        Ok(num_bytes)
    }

    pub fn blocking_read(&mut self, mut buffer: &mut [u8]) -> Result<(), Error> {
        let flags = T::regs().stat().read();

        if flags.or() {
            T::regs().stat().modify(|v| v.set_or(true));
        }

        while !buffer.is_empty() {
            while !T::regs().stat().read().rdrf() {}
            let byte = T::regs().data().read().0;
            buffer[0] = byte as u8;
            buffer = &mut buffer[1..];
        }

        Ok(())
    }

    /// Indicates if the bits are inverted.
    #[inline]
    pub fn is_inverted(&self) -> bool {
        let lpuart = T::regs();
        lpuart.stat().read().rxinv()
    }

    /// Indicates if the FIFO is enabled.
    #[inline]
    pub fn is_fifo_enabled(&self) -> bool {
        let lpuart = T::regs();
        lpuart.fifo().read().rxfe()
    }

    /// Returns the FIFO watermark value.
    #[inline]
    pub fn fifo_watermark(&self) -> u32 {
        let lpuart = T::regs();
        lpuart.water().read().rxwater().into()
    }

    /// Returns the FIFO watermark value.
    #[inline]
    pub fn fifo_flush(&self) {
        let lpuart = T::regs();

        lpuart.fifo().modify(|x| x.set_rxflush(true));
    }
    /// Let the peripheral act as a DMA source.
    ///
    /// After this call, the peripheral will signal to the DMA engine whenever
    /// it has data available to read.
    pub fn enable_dma(&mut self) {
        let lpuart = T::regs();
        lpuart.baud().modify(|b| b.set_rdmae(true));
    }

    /// Stop the peripheral from acting as a DMA source.
    pub fn disable_dma(&mut self) {
        let lpuart = T::regs();
        while lpuart.baud().read().rdmae() {
            lpuart.baud().modify(|b| b.set_rdmae(false));
        }
    }
}

impl<'d, W: dma::Word, T: Instance> dma::Source<W> for LpuartRx<'d, T> {
    fn source_signal(&self) -> u8 {
        T::RX_DREQ
    }

    fn disable_source(&mut self) {
        self.disable_dma();
    }

    fn enable_source(&mut self) {
        self.enable_dma();
    }
    fn source_address(&self) -> *const W {
        let lpuart = T::regs();
        lpuart.data().as_ptr() as _
    }
    fn source_type(&self) -> dma::DmaType {
        dma::DmaType::Hardware
    }
}

pub struct Lpuart<'d, T: Instance> {
    tx: LpuartTx<'d, T>,
    rx: LpuartRx<'d, T>,
}

impl<'d, T: Instance> Lpuart<'d, T> {
    pub fn new(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        rx_dma: Peri<'d, impl crate::dma::Channel>,
        tx_dma: Peri<'d, impl crate::dma::Channel>,
    ) -> Self {
        Self::new_inner(peri, rx, tx, Some(rx_dma.into()), Some(tx_dma.into()))
    }

    fn new_inner(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        rx_dma: Option<Peri<'d, AnyChannel>>,
        tx_dma: Option<Peri<'d, AnyChannel>>,
    ) -> Self {
        let mut res = Self {
            tx: LpuartTx {
                _phantom: &PhantomData,
                tx_dma,
            },
            rx: LpuartRx {
                _phantom: &PhantomData,
                rx_dma,
            },
        };

        res.init();

        res
    }

    fn init(&mut self) {
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        pac::CCM
            .cscdr1()
            .modify(|v| v.set_uart_clk_podf(UartClkPodf::DIVIDE_1));
        pac::CCM.ccgr1().modify(|v| v.set_cg12(0b11));
        //locator(CCGR5, CG12),
        //locator(CCGR0, CG14),
        //locator(CCGR0, CG6),
        //locator(CCGR1, CG12),

        self.reset();
        let lpuart = T::regs();

        lpuart.baud().write(|v| {
            let baudrate = Self::compute_baudrate(80_000_000, 115200);
            v.set_osr(baudrate.osr());
            v.set_sbr(baudrate.sbr());
            v.set_bothedge(baudrate.bothedge());
        });

        lpuart.water().write(|v| {
            v.set_txwater(0b1);
            v.set_rxwater(0b0);
        });

        lpuart.fifo().write(|v| {
            v.set_rxfe(true);
            v.set_txfe(true);
        });

        lpuart.ctrl().write(|v| {
            v.set_pe(false);
            v.set_re(true);
            v.set_te(true);

            v.set_ilt(true);
            v.set_idlecfg(Idlecfg::IDLE_128);
            //v.set_loops(true);
            //v.set_tie(true);
            //v.set_rie(true);
            //v.set_orie(true);
        });

        lpuart.fifo().modify(|v| {
            v.set_rxflush(true);
            v.set_txflush(true);
        });

        lpuart.stat().write_value(lpuart.stat().read());
    }

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(buffer)
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }

    pub fn reset(&mut self) {
        let lpuart = T::regs();
        lpuart.global().write(|v| v.set_rst(true));
        lpuart.global().write(|v| v.set_rst(false));
    }

    pub fn baud(&self) -> Baud {
        let lpuart = T::regs();
        lpuart.baud().read()
    }

    pub const fn compute_baudrate(source_clock_hz: u32, baud: u32) -> Baud {
        const fn max(left: u32, right: u32) -> u32 {
            if left > right {
                left
            } else {
                right
            }
        }
        const fn min(left: u32, right: u32) -> u32 {
            if left < right {
                left
            } else {
                right
            }
        }

        let mut err = u32::max_value();
        let mut best_osr = 0;
        let mut best_sbr = 0;

        let mut osr = 8;
        let mut sbr = 1;
        while osr <= 32 {
            while sbr < 8192 {
                let b = source_clock_hz / (sbr * osr);
                let e = max(baud, b) - min(baud, b);
                if e < err {
                    err = e;
                    best_osr = osr;
                    best_sbr = sbr;
                }
                sbr += 1;
            }
            osr += 1;
        }

        let mut res = Baud(0);
        res.set_osr(pac::lpuart::vals::Osr::from_bits(best_osr as u8 - 1));
        res.set_sbr(best_sbr as u16 - 1);
        res.set_bothedge(4 <= best_osr && best_osr <= 7);
        res
    }

    pub fn parity(&self) -> Parity {
        let lpuart = T::regs();
        let ctrl = lpuart.ctrl().read();

        let pe = ctrl.pe();
        let pt = ctrl.pt();

        if pe {
            if pt {
                Parity::ParityOdd
            } else {
                Parity::ParityEven
            }
        } else {
            Parity::ParityNone
        }
    }

    /// Specify parity bit settings. If there is no parity, use `None`.
    pub fn set_parity(&mut self, parity: Parity) {
        let lpuart = T::regs();

        lpuart.ctrl().modify(|r| match parity {
            Parity::ParityEven => {
                r.set_pe(true);
                r.set_m(true);
                r.set_pt(false);
            }
            Parity::ParityOdd => {
                r.set_pe(true);
                r.set_m(true);
                r.set_pt(true);
            }
            Parity::ParityNone => {
                r.set_pe(false);
                r.set_m(false);
            }
        });
    }

    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        self.rx.read(buffer).await
    }

    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.write(buffer).await
    }

    pub fn read_data(&self) -> u8 {
        let lpuart = T::regs();
        lpuart.data().read().0 as u8
    }

    pub fn data(&self) -> *const usize {
        let lpuart = T::regs();
        lpuart.data().as_ptr() as _
    }

    pub fn write_byte(&mut self, byte: u8) {
        let lpuart = T::regs();
        lpuart.data().write_value(Data(byte as u32));
    }

    pub fn split(self) -> (LpuartTx<'d, T>, LpuartRx<'d, T>) {
        (self.tx, self.rx)
    }
}

impl embedded_io_async::Error for Error {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

impl<'d, T: Instance> embedded_io_async::ErrorType for Lpuart<'d, T> {
    type Error = Error;
}

impl<'d, T: Instance> embedded_io_async::ErrorType for LpuartTx<'d, T> {
    type Error = Error;
}

impl<'d, T: Instance> embedded_io_async::ErrorType for LpuartRx<'d, T> {
    type Error = Error;
}

impl<'d, T: Instance + 'd> embedded_io_async::Read for Lpuart<'d, T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        LpuartRx::<'d, T>::read(&mut self.rx, buf).await
    }
}

impl<'d, T: Instance + 'd> embedded_io_async::Read for LpuartRx<'d, T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        Self::read(self, buf).await
    }
}

impl<'d, T: Instance + 'd> embedded_io_async::Write for Lpuart<'d, T> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        LpuartTx::<'d, T>::write(&mut self.tx, buf).await?;
        Ok(buf.len())
    }
}

impl<'d, T: Instance + 'd> embedded_io_async::Write for LpuartTx<'d, T> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        Self::write(self, buf).await?;
        Ok(buf.len())
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DataBits {
    DataBits7,
    DataBits8,
    DataBits9,
    DataBits10,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,
            invert_rx: false,
            invert_tx: false,
            invert_rts: false,
            invert_cts: false,
            oversampling: 16,
        }
    }
}

mod sealed {
    use core::sync::atomic::AtomicBool;

    use super::*;
    use embassy_sync::waitqueue::AtomicWaker;

    pub trait Mode {}

    pub struct State {
        pub tx_waker: AtomicWaker,
        pub rx_waker: AtomicWaker,
        pub idle_irq: AtomicBool,
    }

    impl State {
        pub const fn new() -> Self {
            Self {
                tx_waker: AtomicWaker::new(),
                rx_waker: AtomicWaker::new(),
                idle_irq: AtomicBool::new(false),
            }
        }
    }

    pub trait Instance {
        const TX_DREQ: u8;
        const RX_DREQ: u8;

        type Interrupt: interrupt::typelevel::Interrupt;

        fn regs() -> pac::lpuart::Lpuart;

        fn state() -> &'static State;

        //#[cfg(feature = "nightly")]
        //fn buffered_state() -> &'static buffered::State;

        ////fn dma_state() -> &'static DmaState;
    }

    pub trait TxPin<T: Instance> {}
    pub trait RxPin<T: Instance> {}
    pub trait CtsPin<T: Instance> {}
    pub trait RtsPin<T: Instance> {}
}

pub trait Mode: sealed::Mode {}

pub trait Instance: PeripheralType + sealed::Instance + 'static + Send {}

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

macro_rules! impl_instance {
    ($inst:ident, $irq:ident, $n:expr, $tx_dreq:expr, $rx_dreq:expr) => {
        impl sealed::Instance for peripherals::$inst {
            type Interrupt = crate::interrupt::typelevel::$irq;

            const TX_DREQ: u8 = $tx_dreq;
            const RX_DREQ: u8 = $rx_dreq;

            fn regs() -> pac::lpuart::Lpuart {
                pac::$inst
            }

            fn state() -> &'static crate::lpuart::sealed::State {
                static STATE: crate::lpuart::sealed::State = crate::lpuart::sealed::State::new();
                &STATE
            }
        }
        impl Instance for peripherals::$inst {}
    };
}

//    pub(super) const LPUART_DMA_TX_MAPPING: [u32; 8] = [2, 66, 4, 68, 6, 70, 8, 72];
//    pub(super) const LPUART_DMA_RX_MAPPING: [u32; 8] = [3, 67, 5, 69, 7, 71, 9, 73];
impl_instance!(LPUART1, LPUART1, 1, 2, 3);
impl_instance!(LPUART2, LPUART2, 2, 66, 67);
impl_instance!(LPUART3, LPUART3, 3, 4, 5);
impl_instance!(LPUART4, LPUART4, 4, 68, 69);

//impl_instance!(LPUART5, LPUART5, 5, 6, 7);
//impl_instance!(LPUART6, LPUART6, 6, 70, 71);
//impl_instance!(LPUART7, LPUART7, 7, 8, 9);
//impl_instance!(LPUART8, LPUART8, 8, 72, 73);

pub trait TxPin<T: Instance>: PeripheralType + sealed::TxPin<T> {}
pub trait RxPin<T: Instance>: PeripheralType + sealed::RxPin<T> {}
pub trait CtsPin<T: Instance>: sealed::CtsPin<T> {}
pub trait RtsPin<T: Instance>: sealed::RtsPin<T> {}

macro_rules! impl_pin {
    ($pin:ident, $instance:ident, $n:expr, $function:ident) => {
        impl sealed::$function<peripherals::$instance> for peripherals::$pin {}
        impl $function<peripherals::$instance> for peripherals::$pin {}
    };
}

impl_pin!(GPIO_08, LPUART1, 1, CtsPin);
impl_pin!(GPIO_07, LPUART1, 1, RtsPin);
impl_pin!(GPIO_09, LPUART1, 1, RxPin);
impl_pin!(GPIO_SD_11, LPUART1, 1, RxPin);
impl_pin!(GPIO_10, LPUART1, 1, TxPin);
impl_pin!(GPIO_SD_12, LPUART1, 1, TxPin);
impl_pin!(GPIO_AD_08, LPUART2, 2, CtsPin);
impl_pin!(GPIO_AD_07, LPUART2, 2, RtsPin);
impl_pin!(GPIO_13, LPUART2, 2, RxPin);
impl_pin!(GPIO_SD_09, LPUART2, 2, RxPin);
impl_pin!(GPIO_AD_00, LPUART2, 2, TxPin);
impl_pin!(GPIO_SD_10, LPUART2, 2, TxPin);
impl_pin!(GPIO_AD_14, LPUART3, 3, CtsPin);
impl_pin!(GPIO_AD_13, LPUART3, 3, RtsPin);
impl_pin!(GPIO_11, LPUART3, 3, RxPin);
impl_pin!(GPIO_AD_07, LPUART3, 3, RxPin);
impl_pin!(GPIO_07, LPUART3, 3, RxPin);
impl_pin!(GPIO_12, LPUART3, 3, TxPin);
impl_pin!(GPIO_AD_08, LPUART3, 3, TxPin);
impl_pin!(GPIO_08, LPUART3, 3, TxPin);
impl_pin!(GPIO_AD_14, LPUART4, 4, CtsPin);
impl_pin!(GPIO_AD_13, LPUART4, 4, RtsPin);
impl_pin!(GPIO_AD_01, LPUART4, 4, RxPin);
impl_pin!(GPIO_05, LPUART4, 4, RxPin);
impl_pin!(GPIO_AD_02, LPUART4, 4, TxPin);
impl_pin!(GPIO_06, LPUART4, 4, TxPin);
