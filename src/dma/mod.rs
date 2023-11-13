use crate::interrupt::typelevel::Interrupt;
use crate::interrupt::InterruptExt;
use crate::{interrupt, into_ref, pac, peripherals, Peripheral, PeripheralRef};
use core::future::Future;
use core::mem::size_of;
use core::pin::Pin;
use core::task::{Context, Poll};
use embassy_hal_internal::impl_peripheral;
use embassy_sync::waitqueue::AtomicWaker;

pub mod channel;
pub mod dmamux;
pub mod ringbuffer;

pub fn init() {
    pac::CCM.ccgr5().modify(|r| {
        r.set_cg3(0b11);
    });
    unsafe {
        crate::interrupt::typelevel::DMA0::enable();
        crate::interrupt::typelevel::DMA0::unpend();
        crate::interrupt::typelevel::DMA1::enable();
        crate::interrupt::typelevel::DMA1::unpend();
        crate::interrupt::typelevel::DMA2::enable();
        crate::interrupt::typelevel::DMA2::unpend();
        crate::interrupt::typelevel::DMA3::enable();
        crate::interrupt::typelevel::DMA3::unpend();
        crate::interrupt::typelevel::DMA4::enable();
        crate::interrupt::typelevel::DMA4::unpend();
        crate::interrupt::typelevel::DMA5::enable();
        crate::interrupt::typelevel::DMA5::unpend();
        crate::interrupt::typelevel::DMA6::enable();
        crate::interrupt::typelevel::DMA6::unpend();
        crate::interrupt::typelevel::DMA7::enable();
        crate::interrupt::typelevel::DMA7::unpend();
        crate::interrupt::typelevel::DMA8::enable();
        crate::interrupt::typelevel::DMA8::unpend();
        crate::interrupt::typelevel::DMA9::enable();
        crate::interrupt::typelevel::DMA9::unpend();
        crate::interrupt::typelevel::DMA10::enable();
        crate::interrupt::typelevel::DMA10::unpend();
        crate::interrupt::typelevel::DMA11::enable();
        crate::interrupt::typelevel::DMA11::unpend();
        crate::interrupt::typelevel::DMA10::enable();
        crate::interrupt::typelevel::DMA12::unpend();
        crate::interrupt::typelevel::DMA13::enable();
        crate::interrupt::typelevel::DMA13::unpend();
        crate::interrupt::typelevel::DMA14::enable();
        crate::interrupt::typelevel::DMA14::unpend();
        crate::interrupt::typelevel::DMA15::enable();
        crate::interrupt::typelevel::DMA15::unpend();

        crate::interrupt::typelevel::DMA_ERROR::enable();
        crate::interrupt::typelevel::DMA_ERROR::unpend();

        for i in 0..CHANNEL_COUNT {
            let tcd = pac::DMA0.tcd_x(i);
            tcd.saddr().as_ptr().write_volatile(0);
            tcd.daddr().as_ptr().write_volatile(0);

            (tcd.soff().as_ptr() as *mut u16).write_volatile(0);
            (tcd.doff().as_ptr() as *mut u16).write_volatile(0);
            (tcd.attr().as_ptr() as *mut u16).write_volatile(0);

            tcd.nbytes_mlno().as_ptr().write_volatile(0);
            tcd.slast().as_ptr().write_volatile(0);
            (tcd.citer_elinkno().as_ptr() as *mut u16).write_volatile(0);
            (tcd.biter_elinkno().as_ptr() as *mut u16).write_volatile(0);
            tcd.dlastsga().as_ptr().write_volatile(0);
            (tcd.csr().as_ptr() as *mut u16).write_volatile(0);
        }
    }
}

pub trait Channel:
    Peripheral<P = Self> + sealed::Channel + Into<AnyChannel> + Sized + 'static
{
    fn number(&self) -> u8;

    fn tcd(&self) -> pac::dma::Tcd;

    /// Set the source address for a DMA transfer
    ///
    /// `saddr` should be a memory location that can provide the DMA controller
    /// with data.
    ///
    /// # Safety
    ///
    /// If the DMA channel is already enabled, the DMA engine may start reading this
    /// memory location. You must ensure that reads to `saddr` do not perform
    /// inappropriate side effects. You must ensure `saddr` is valid for the
    /// lifetime of the transfer.
    fn set_source_address<W: Word>(&self, saddr: *const W) {
        self.tcd().saddr().write_value(saddr as u32);
    }

    /// Set the source offset *in bytes*
    ///
    /// `offset` could be negative, which would decrement the address.
    ///
    /// # Safety
    ///
    /// This method could allow a DMA engine to read beyond a buffer or
    /// address. You must ensure that the source is valid for these offsets.
    fn set_source_offset(&self, offset: i16) {
        let soff = self.tcd().soff().as_ptr() as *mut u16;
        unsafe { soff.write_volatile(offset as u16) };
    }

    fn set_source_attributes<W: Word>(&self, modulo: u8) {
        let wsize = W::size();

        unsafe {
            let attr = self.tcd().attr().as_ptr() as *mut u16;
            let mut newattr = TcdAttr(attr.read_volatile() as u32);
            newattr.set_smod(modulo);
            newattr.set_ssize(wsize.into());

            attr.write_volatile(newattr.0 as u16);
        };
    }

    fn set_source_last_address_adjustment(&self, adjustment: i32) {
        self.tcd().slast().write_value(adjustment as u32);
    }

    /// Set the destination address for a DMA transfer
    ///
    /// `daddr` should be a memory location that can store data from the
    /// DMA controller.
    ///
    /// # Safety
    ///
    /// If the DMA channel is already enabled, the DMA engine may start
    /// writing to this address. You must ensure that writes to `daddr`
    /// are safe, and that the memory is valid for the lifetime of the
    /// transfer.
    fn set_destination_address<W: Word>(&self, daddr: *const W) {
        self.tcd().daddr().write_value(daddr as u32);
    }

    fn previous_destination_address(&self) -> *const u8 {
        self.tcd().daddr().read() as *const u8
    }

    fn set_destination_last_address_adjustment(&self, adjustment: i32) {
        self.tcd().dlastsga().write_value(adjustment as u32);
    }

    fn set_destination_attributes<W: Word>(&self, modulo: u8) {
        let wsize = W::size();

        unsafe {
            let attr = self.tcd().attr().as_ptr() as *mut u16;
            let mut newattr = TcdAttr(attr.read_volatile() as u32);
            newattr.set_dmod(modulo);
            newattr.set_dsize(wsize);

            attr.write_volatile(newattr.0 as u16);
        };
    }

    /// Set the destination offset *in bytes*
    ///
    /// `offset` could be negative, which would decrement the address.
    ///
    /// # Safety
    ///
    /// This method could allow a DMA engine to write beyond the range of
    /// a buffer. You must ensure that the destination is valid for these
    /// offsets.
    fn set_destination_offset(&self, offset: i16) {
        let doff = self.tcd().doff().as_ptr() as *mut u16;
        unsafe { doff.write_volatile(offset as u16) };
    }

    fn degrade(self) -> AnyChannel {
        AnyChannel {
            number: self.number(),
        }
    }

    fn enable(&self) {
        self.set_interrupt_on_error();
        self.set_interrupt_on_completion(true);

        let serq = pac::DMA0.serq().as_ptr() as *mut u8;
        unsafe { serq.write_volatile(self.number()) };
    }

    fn reset(&mut self) {
        use pac::dma::regs::*;
        self.tcd().saddr().write_value(0);
        self.tcd().daddr().write_value(0);
        self.tcd().soff().write_value(TcdSoff(0));
        self.tcd().attr().write_value(TcdAttr(0));
        self.tcd().nbytes_mlno().write_value(0);
        self.tcd().slast().write_value(0);
        self.tcd().citer_elinkno().write_value(TcdCiterElinkno(0));
        self.tcd().dlastsga().write_value(0);
        self.tcd().doff().write_value(TcdDoff(0));
        self.tcd().csr().write_value(TcdCsr(0));
        self.tcd().biter_elinkno().write_value(TcdBiterElinkno(0));
    }

    fn is_enabled(&self) -> bool {
        pac::DMA0.erq().read().erq(self.number() as usize)
    }

    fn disable(&self) {
        let cerq = pac::DMA0.cerq().as_ptr() as *mut u8;
        unsafe { cerq.write_volatile(self.number()) };
    }

    fn is_interrupted(&self) -> bool {
        pac::DMA0.int().read().int(self.number() as usize)
    }

    fn clear_interrupt(&self) {
        pac::DMA0.cint().write(|x| x.set_cint(self.number()));
    }

    fn set_disable_on_completion(&mut self, dreq: bool) {
        self.tcd().csr().modify(|csr| {
            csr.set_dreq(dreq);
        })
    }

    fn set_interrupt_on_completion(&self, intr: bool) {
        self.tcd().csr().modify(|csr| csr.set_intmajor(intr))
    }

    fn set_interrupt_on_error(&self) {
        let seei = pac::DMA0.seei().as_ptr() as *mut u8;
        unsafe { seei.write_volatile(self.number()) };
    }

    fn is_complete(&self) -> bool {
        self.tcd().csr().read().done()
    }

    fn clear_complete(&self) {
        let cdne = pac::DMA0.cdne().as_ptr() as *mut u8;
        unsafe { cdne.write_volatile(self.number()) };
    }

    fn is_error(&self) -> bool {
        pac::DMA0.err().read().err(self.number() as usize)
    }

    fn clear_error(&self) {
        let cerr = pac::DMA0.cerr().as_ptr() as *mut u8;
        unsafe { cerr.write_volatile(self.number()) };
    }

    fn is_active(&self) -> bool {
        self.tcd().csr().read().active()
    }

    fn is_hardware_signaling(&self) -> bool {
        pac::DMA0.hrs().read().hrs(self.number() as usize)
    }

    fn start(&self) {
        let ssrt = pac::DMA0.ssrt().as_ptr() as *mut u8;

        unsafe { ssrt.write_volatile(self.number()) };
    }

    fn set_minor_loop_bytes(&self, nbytes: u32) {
        self.tcd().nbytes_mlno().write_value(nbytes);
    }

    fn set_transfer_iterations(&mut self, iterations: u16) {
        let tcd = self.tcd();

        unsafe {
            let reg = tcd.citer_elinkno().as_ptr() as *mut u16;
            reg.write_volatile(iterations);

            let reg = tcd.biter_elinkno().as_ptr() as *mut u16;
            reg.write_volatile(iterations);
        }
    }

    fn set_configuration(&mut self, configuration: Configuration) {
        let chcfg = pac::DMAMUX.chcfg(self.number() as usize);
        match configuration {
            Configuration::Off => chcfg.write_value(Chcfg(0)),
            Configuration::Enable { source, periodic } => {
                chcfg.write(|v| {
                    v.set_source(source);
                    v.set_enbl(true);
                    if periodic {
                        assert!(
                            self.number() < 4,
                            "Requested DMA periodic triggering on an unsupported channel."
                        );
                        v.set_trig(true);
                    }
                });
            }
            Configuration::AlwaysOn => {
                // See note in reference manual: when A_ON is high, SOURCE is ignored.
                chcfg.write(|v| {
                    v.set_enbl(true);
                    v.set_a_on(true);
                });
            }
        }
    }
}

pub struct AnyChannel {
    number: u8,
}

impl sealed::Channel for AnyChannel {}

impl_peripheral!(AnyChannel);

impl Channel for AnyChannel {
    fn number(&self) -> u8 {
        self.number
    }

    fn tcd(&self) -> pac::dma::Tcd {
        pac::DMA0.tcd_x(self.number as usize)
    }
}

macro_rules! channel {
    ($name:ident, $num:expr, $irq: ident) => {
        impl sealed::Channel for peripherals::$name {}
        impl Channel for peripherals::$name {
            fn number(&self) -> u8 {
                $num
            }

            fn tcd(&self) -> pac::dma::Tcd {
                pac::DMA0.tcd_x($num)
            }
        }

        impl From<peripherals::$name> for crate::dma::AnyChannel {
            fn from(val: peripherals::$name) -> Self {
                crate::dma::Channel::degrade(val)
            }
        }

        #[interrupt]
        fn $irq() {
            CHANNEL_WAKERS[$num].wake();

            let cint = pac::DMA0.cint().as_ptr() as *mut u8;
            unsafe { cint.write_volatile($num) };
        }
    };
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a, C: Channel> {
    pub channel: PeripheralRef<'a, C>,
    pub software: bool,
}

impl<'a, C: Channel> Transfer<'a, C> {
    pub(crate) fn new(channel: impl Peripheral<P = C> + 'a) -> Self {
        into_ref!(channel);

        Self {
            channel,
            software: false,
        }
    }
}

impl<'a, C: Channel> Unpin for Transfer<'a, C> {}

impl<'a, C: Channel> Future for Transfer<'a, C> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        CHANNEL_WAKERS[self.channel.number() as usize].register(cx.waker());

        // This driver is only expecting to catch synchronous errors
        // (those that manifest once we enable the transfer). If there
        // is a misconfiguration that only the hardware detects, we expect
        // to see it as soon as we loop back around after the enable.
        if self.channel.is_error() {
            //let es = self.channel.error_status();
            //self.channel.clear_error();
            //return Poll::Ready(Err(es));
        }

        if self.channel.is_complete() {
            self.channel.clear_complete();
            return Poll::Ready(());
        }

        if self.channel.is_enabled() {
            return Poll::Pending;
        }

        self.channel.enable();

        if self.software {
            self.channel.start();
            self.software = false;
        }

        Poll::Pending
    }
}

impl<'a, C: Channel> Drop for Transfer<'a, C> {
    fn drop(&mut self) {
        self.channel.disable();
        self.channel.clear_complete();
        self.channel.clear_error();
    }
}
#[interrupt]
fn DMA_ERROR() {
    let int = pac::DMA0.int().read().0 as u16;
    let es = pac::DMA0.es().read();
    let dbe = es.dbe();
    let sbe = es.sbe();
    let sge = es.sge();
    let nce = es.nce();
    let doe = es.doe();
    let dae = es.dae();
    let soe = es.soe();
    let sae = es.sae();
    let errchn = es.errchn();
    let cpe = es.cpe();

    let err_val = pac::DMA0.err().read().0 as u16;

    let cerr = pac::DMA0.cerr().as_ptr() as *mut u8;
    unsafe { cerr.write_volatile(0xff) };
}

pub unsafe fn write<'a, C: Channel, D: Destination<W>, W: Word>(
    ch: impl Peripheral<P = C> + 'a,
    buffer: &[W],
    destination: &mut D,
) -> Transfer<'a, C> {
    into_ref!(ch);

    ch.disable();
    ch.set_disable_on_completion(true);

    ch.set_configuration(Configuration::enable(destination.destination_signal()));

    set_source_linear_buffer(&ch, buffer);

    ch.set_destination_address(destination.destination_address());
    ch.set_destination_offset(0);
    ch.set_destination_attributes::<W>(0);
    ch.set_destination_last_address_adjustment(0);

    ch.set_minor_loop_bytes(core::mem::size_of::<W>() as u32);
    ch.set_transfer_iterations(buffer.len() as u16);

    destination.enable_destination();

    Transfer {
        channel: ch,
        software: false,
    }
}

pub unsafe fn copy<'a, C: Channel, W: Word>(
    ch: impl Peripheral<P = C> + 'a,
    from: &[W],
    to: &mut [W],
) -> Transfer<'a, C> {
    into_ref!(ch);

    ch.disable();
    ch.set_disable_on_completion(true);

    set_source_linear_buffer(&ch, from);
    set_destination_linear_buffer(&ch, to);

    ch.set_configuration(Configuration::Off);

    ch.set_minor_loop_bytes(
        core::mem::size_of::<W>().saturating_mul(from.len().min(to.len())) as u32,
    );
    ch.set_transfer_iterations(1);

    Transfer {
        channel: ch,
        software: true,
    }
}

//pub unsafe fn pipe<'a, C: Channel, S: Source<W>, D: Destination<W>, W: Word>(
//    ch: impl Peripheral<P = C> + 'a,
//    source: &S,
//    destination: &mut D,
//    len: usize,
//) -> Transfer<'a, C> {
//    into_ref!(ch);
//
//    ch.disable();
//    ch.set_disable_on_completion(true);
//
//    match (source.source_type(), destination.destination_type()) {
//        (LinearBuffer(usize), LinearBuffer(usize)) => {
//            //
//        }
//
//        (LinearBuffer(usize), CircularBuffer(usize)) => {
//            //
//        }
//
//        (LinearBuffer(usize), Hardware(usize)) => {
//            //
//        }
//
//        (CircularBuffer(usize), CircularBuffer(usize)) => {
//            //
//        }
//
//        (CircularBuffer(usize), LinearBuffer(usize)) => {
//            //
//        }
//
//
//        (Hardware, Hardware) => {
//            //
//        }
//    }
//
//    ///////set_source_linear_buffer(&ch, from);
//    ///////set_destination_linear_buffer(&ch, to);
//    ///////ch.set_configuration(Configuration::Off);
//    ///////ch.set_minor_loop_bytes(
//    ///////    core::mem::size_of::<W>().saturating_mul(from.len().min(to.len())) as u32,
//    ///////);
//    ///////ch.set_transfer_iterations(1);
//
//    Transfer {
//        channel: ch,
//        software: true,
//    }
//}

pub unsafe fn read<'a, C: Channel, S: Source<W>, W: Word>(
    ch: impl Peripheral<P = C> + 'a,
    source: &mut S,
    buffer: &mut [W],
) -> Transfer<'a, C> {
    into_ref!(ch);

    ch.disable();
    ch.set_disable_on_completion(true);

    ch.set_configuration(Configuration::enable(source.source_signal()));

    unsafe {
        ch.set_source_address(source.source_address());
        ch.set_source_offset(0);
        ch.set_source_attributes::<W>(0);
        ch.set_source_last_address_adjustment(0);

        set_destination_linear_buffer(&ch, buffer);
        ch.set_minor_loop_bytes(core::mem::size_of::<W>() as u32);
        ch.set_transfer_iterations(buffer.len() as u16);
    }

    source.enable_source();

    Transfer {
        channel: ch,
        software: false,
    }
}

pub unsafe fn set_source_linear_buffer<'a, C: Channel, W: Word>(
    chan: &PeripheralRef<'a, C>,
    source: &[W],
) {
    chan.set_source_address(source.as_ptr());
    chan.set_source_offset(core::mem::size_of::<W>() as i16);
    chan.set_source_attributes::<W>(0);
    let adj = ((source.len() * core::mem::size_of::<W>()) as i32).wrapping_neg();
    chan.set_source_last_address_adjustment(adj);
}

mod sealed {
    pub use crate::pac::dma::Tcd;

    pub trait Channel {}
    pub trait Word {}
}

pub unsafe fn set_destination_linear_buffer<'a, C: Channel, W: Word>(
    chan: &PeripheralRef<'a, C>,
    destination: &mut [W],
) {
    chan.set_destination_address(destination.as_ptr());
    chan.set_destination_offset(core::mem::size_of::<W>() as i16);
    chan.set_destination_attributes::<W>(0);

    chan.set_destination_last_address_adjustment(
        (core::mem::size_of_val(destination) as i32).wrapping_neg(),
    );
}

/// Assert properties about the circular buffer
fn circular_buffer_asserts<W>(buffer: &[W]) {
    let len = buffer.len();
    assert!(
        len.is_power_of_two(),
        "DMA circular buffer size is not power of two"
    );
    let start = buffer.as_ptr();
    let size = len * core::mem::size_of::<W>();
    assert!(
        (start as usize) % size == 0,
        "DMA circular buffer is not properly aligned"
    );
}

/// Compute the circular buffer modulo value
pub fn circular_buffer_modulo<W>(buffer: &[W]) -> u32 {
    31 - (buffer.len() * core::mem::size_of::<W>()).leading_zeros()
}

/// Set a circular buffer as the source for a DMA transfer
///
/// When the transfer completes, the DMA channel remain at the
/// next element in the circular buffer.
///
/// # Safety
///
/// Caller must ensure that the source is valid for the lifetime of the transfer,
/// and for all subsequent transfers performed by this DMA channel with this buffer.
///
/// # Panics
///
/// Panics if
///
/// - the capacity is not a power of two
/// - the alignment is not a multiple of the buffer's size in bytes
pub unsafe fn set_source_circular_buffer<'a, C: Channel, W: Word>(
    chan: &PeripheralRef<'a, C>,
    source: &[W],
) {
    circular_buffer_asserts(source);
    let modulo = circular_buffer_modulo(source);

    chan.set_source_address(source.as_ptr());
    chan.set_source_offset(core::mem::size_of::<W>() as i16);
    chan.set_source_attributes::<W>(modulo as u8);
    chan.set_source_last_address_adjustment(0);
}

/// Set a circular buffer as the destination for a DMA transfer
///
/// When the transfer completes, the DMA channel remain at the
/// next element in the circular buffer.
///
/// # Safety
///
/// Caller must ensure that the destination is valid for the lifetime of the transfer,
/// and for all subsequent transfers performed by this DMA channel with this buffer.
///
/// # Panics
///
/// Panics if
///
/// - the capacity is not a power of two
/// - the alignment is not a multiple of the buffer's size in bytes

pub unsafe fn set_destination_circular_buffer<'a, C: Channel, W: Word>(
    chan: &PeripheralRef<'a, C>,
    destination: &mut [W],
) {
    circular_buffer_asserts(destination);
    let modulo = circular_buffer_modulo(destination);

    chan.set_destination_address(destination.as_ptr());
    chan.set_destination_offset(core::mem::size_of::<W>() as i16);
    chan.set_destination_attributes::<W>(modulo as u8);
    chan.set_destination_last_address_adjustment(0);
}

pub trait Word: sealed::Word {
    fn size() -> u8;
}

impl sealed::Word for u8 {}
impl Word for u8 {
    fn size() -> u8 {
        0
    }
}

impl sealed::Word for u16 {}
impl Word for u16 {
    fn size() -> u8 {
        1
    }
}

impl sealed::Word for u32 {}
impl Word for u32 {
    fn size() -> u8 {
        2
    }
}

impl sealed::Word for u64 {}
impl Word for u64 {
    fn size() -> u8 {
        3
    }
}

impl sealed::Word for [u8; 32] {}
impl Word for [u8; 32] {
    fn size() -> u8 {
        5
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Configuration {
    /// The DMAMUX channel is disabled
    Off,
    /// The DMAMUX is enabled, permitting hardware triggering.
    /// See [`enable`](Configuration::enable) to enable
    /// the channel without periodic triggering.
    Enable {
        /// The DMA channel source (slot number)
        ///
        /// Specifies which DMA source is routed to the DMA channel.
        source: u8,
        /// Set the periodic triggering flag to schedule DMA transfers on PIT
        /// timer scheduling.
        ///
        /// `periodic` only works for the first four DMA channels, since
        /// it corresponds to the PIT timers.
        periodic: bool,
    },
    /// The DMAMUX is always on, and there's no need for software
    /// or hardware activation
    ///
    /// Use `AlwaysOn` for
    /// - memory-to-memory transfers
    /// - memory to external bus transfers
    AlwaysOn,
}

impl Configuration {
    /// Enable the channel without triggering
    ///
    /// Shorthand for `ChannelConfiguration::Enable { source, periodic: false }`.
    /// Use `enable()` to avoid possible panics in
    /// [`set_configuration`](crate::channel::Channel::set_configuration).
    pub const fn enable(source: u8) -> Self {
        Configuration::Enable {
            source,
            periodic: false,
        }
    }
}

/// A peripheral that can be the source of DMA data
///
/// By 'source,' we mean that it provides data for a DMA transfer.
/// A source would be a hardware device writing data into memory,
/// like a UART receiver.
///
/// # Safety
///
/// `Source` should only be implemented on peripherals that are
/// DMA capable. This trait should be implemented by HAL authors
/// who are exposing DMA capable peripherals.
#[auto_impl::auto_impl(&mut)]
pub trait Source<W: Word> {
    /// Peripheral source request signal
    ///
    /// See Table 4-3 of the reference manual. A source may
    /// has a qualifier like 'receive' in the name.
    fn source_signal(&self) -> u8 {
        0
    }
    /// Returns a pointer to the register from which the DMA channel
    /// reads data
    ///
    /// This is the register that software reads to acquire data from
    /// a device. The type of the pointer describes the type of reads
    /// the DMA channel performs when transferring data.
    ///
    /// This memory is assumed to be static. Repeated `source` calls
    /// should always return the same address.
    fn source_address(&self) -> *const W;
    /// Perform any actions necessary to enable DMA transfers
    ///
    /// Callers use this method to put the peripheral in a state where
    /// it can supply the DMA channel with data.
    fn enable_source(&mut self) {}
    /// Perform any actions necessary to disable or cancel DMA transfers
    ///
    /// This may include undoing the actions in `enable_source`.
    fn disable_source(&mut self) {}

    fn source_type(&self) -> DmaType;

    fn source_offset(&self) -> u8 {
        0
    }

    fn source_modulo(&self) -> u8 {
        0
    }
}

impl<W: Word> Source<W> for [W] {
    fn source_signal(&self) -> u8 {
        0
    }
    fn source_address(&self) -> *const W {
        self.as_ptr()
    }
    fn source_type(&self) -> DmaType {
        DmaType::LinearBuffer(self.len())
    }
    fn source_offset(&self) -> u8 {
        size_of::<W>() as _
    }
}

impl<W: Word> Source<W> for &[W] {
    fn source_signal(&self) -> u8 {
        0
    }
    fn source_address(&self) -> *const W {
        self.as_ptr()
    }
    fn source_type(&self) -> DmaType {
        DmaType::LinearBuffer(self.len())
    }
    fn source_offset(&self) -> u8 {
        size_of::<W>() as _
    }
}

pub enum DmaType {
    LinearBuffer(usize),
    CircularBuffer(usize),
    Hardware,
}

/// A peripheral that can be the destination for DMA data
///
/// By 'destination,' we mean that it receives data from a DMA transfer.
/// A destination would be a peripheral that could send data out of
/// processor memory, like a UART transmitter.
///
/// # Safety
///
/// `Destination` should only be implemented on peripherals that are
/// DMA capable. This trait should be implemented by HAL authors
/// who are exposing DMA capable peripherals.
#[auto_impl::auto_impl(&mut)]
pub trait Destination<W: Word> {
    /// Peripheral destination request signal
    ///
    /// See Table 4-3 of the reference manual. A destination mave
    /// has a qualifier like 'transfer' in the name.
    fn destination_signal(&self) -> u8 {
        0
    }
    /// Returns a pointer to the register into which the DMA channel
    /// writes data
    ///
    /// This is the register that software writes to when sending data to a
    /// device. The type of the pointer describes the type of reads the
    /// DMA channel performs when transferring data.
    fn destination_address(&self) -> *const W;
    /// Perform any actions necessary to enable DMA transfers
    ///
    /// Callers use this method to put the peripheral into a state where
    /// it can accept transfers from a DMA channel.
    fn enable_destination(&mut self) {}
    /// Perform any actions necessary to disable or cancel DMA transfers
    ///
    /// This may include undoing the actions in `enable_destination`.
    fn disable_destination(&mut self) {}

    fn destination_type(&self) -> DmaType;

    fn destination_offset(&self) -> u8 {
        0
    }

    fn destination_modulo(&self) -> u8 {
        0
    }
}

impl<W: Word> Destination<W> for [W] {
    fn destination_address(&self) -> *const W {
        self.as_ptr()
    }
    fn destination_type(&self) -> DmaType {
        DmaType::LinearBuffer(self.len())
    }
    fn destination_offset(&self) -> u8 {
        size_of::<W>() as _
    }
}

impl<W: Word> Destination<W> for &[W] {
    fn destination_address(&self) -> *const W {
        self.as_ptr()
    }
    fn destination_type(&self) -> DmaType {
        DmaType::LinearBuffer(self.len())
    }
    fn destination_offset(&self) -> u8 {
        size_of::<W>() as _
    }
}

channel!(DMA_CH0, 0, DMA0);
channel!(DMA_CH1, 1, DMA1);
channel!(DMA_CH2, 2, DMA2);
channel!(DMA_CH3, 3, DMA3);
channel!(DMA_CH4, 4, DMA4);
channel!(DMA_CH5, 5, DMA5);
channel!(DMA_CH6, 6, DMA6);
channel!(DMA_CH7, 7, DMA7);
channel!(DMA_CH8, 8, DMA8);
channel!(DMA_CH9, 9, DMA9);
channel!(DMA_CH10, 10, DMA10);
channel!(DMA_CH11, 11, DMA11);
channel!(DMA_CH12, 12, DMA12);
channel!(DMA_CH13, 13, DMA13);
channel!(DMA_CH14, 14, DMA14);
channel!(DMA_CH15, 15, DMA15);

pub const CHANNEL_COUNT: usize = 16;

const NEW_AW: AtomicWaker = AtomicWaker::new();
static CHANNEL_WAKERS: [AtomicWaker; CHANNEL_COUNT] = [NEW_AW; CHANNEL_COUNT];

use pac::dma::regs::TcdAttr;
use pac::dma::vals::TcdAttrSsize;
use pac::dmamux::regs::Chcfg;
