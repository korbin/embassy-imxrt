use crate::interrupt::typelevel::{Binding, Interrupt};
use bitvec::prelude::*;
use core::{
    future::poll_fn,
    marker::PhantomData,
    sync::atomic::{compiler_fence, Ordering},
    task::Poll,
};
use driver::{EndpointIn, EndpointOut};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver as driver;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType,
    Event, Unsupported,
};
use portable_atomic::AtomicBool;

use crate::{
    interrupt, pac,
    pac::{usb::regs::*, usbphy::regs::*},
    peripherals,
};

pub mod endpoint;
use endpoint::*;

pub mod buffer;
pub mod cache;
pub mod qh;
pub mod state;
pub mod td;
pub mod vcell;

//////1. After Host drive is IDLE for 3ms, an SLI interrupt is issued (the "DCSUSPEND" or
//////"SLI" bit in USB_USBSTS)
//////2. Set the "PHCD" bit on USB_PORTSC1
//////3. Set all PWD bits in USBPHYx_PWD
//////4. Set CLKGATE in USBPHYx_CTRL
//////NOTE
//////Step 2,3,4 shall be done in atomic operation. That is, interrupt
//////should be disabled during these three steps.

const EP_COUNT: usize = 8;

const NEW_AW: AtomicWaker = AtomicWaker::new();

static BUS_WAKER: AtomicWaker = NEW_AW;
pub static EP_IN_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];
pub static EP_OUT_WAKERS: [AtomicWaker; EP_COUNT] = [NEW_AW; EP_COUNT];

const NEW_AB: AtomicBool = AtomicBool::new(false);
static IRQ_PORT_CHANGE: AtomicBool = NEW_AB;
static IRQ_RESET: AtomicBool = NEW_AB;
static IRQ_SUSPEND: AtomicBool = NEW_AB;
static IRQ_RESUME: AtomicBool = AtomicBool::new(false);

pub struct InterruptHandler<T: Instance> {
    _usb: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::ControllerInterrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let status = T::regs().usbsts().read();

        let usb_error = status.uei() || status.sei();
        if usb_error {
            //defmt::println!("!!!!!!! USB ERROR INTERRUPT !!!!!!!!");
        }

        let reset = status.uri();
        if reset {
            IRQ_RESET.store(true, Ordering::Relaxed);
            T::regs().usbsts().modify(|v| v.set_uri(true));

            BUS_WAKER.wake();
        }

        let port_change = status.pci();
        if port_change {
            let resume = T::regs().portsc1().read().fpr();
            let suspend = T::regs().portsc1().read().susp();

            if resume {
                IRQ_RESUME.store(true, Ordering::Relaxed);
            } else if suspend {
                IRQ_SUSPEND.store(true, Ordering::Relaxed);
            } else {
                IRQ_PORT_CHANGE.store(true, Ordering::Relaxed);
            }
            T::regs().usbsts().modify(|v| v.set_pci(true));

            BUS_WAKER.wake();
        }

        let suspend = status.sli();
        if suspend {
            T::regs().usbsts().modify(|v| v.set_sli(true));

            BUS_WAKER.wake();
        }

        let ep_setup = T::regs().endptsetupstat().read().endptsetupstat();
        let has_setup = ep_setup & 1 != 0;

        if has_setup {
            EP_OUT_WAKERS[0].wake();
        }

        let usb_interrupt = status.ui();
        if usb_interrupt {
            T::regs().usbsts().modify(|v| v.set_ui(true));

            let endptcomplete = T::regs().endptcomplete().read();

            let ep_out_complete = endptcomplete.erce();
            for (i, bit) in (ep_out_complete.view_bits::<Lsb0>().into_iter()).enumerate() {
                if *bit {
                    EP_OUT_WAKERS[i].wake();
                }
            }

            let ep_in_complete = endptcomplete.etce();
            for (i, bit) in (ep_in_complete.view_bits::<Lsb0>().into_iter()).enumerate() {
                if *bit {
                    EP_IN_WAKERS[i].wake();
                }
            }
        }
    }
}

pub struct Driver<'d, T: Instance> {
    _phantom: PhantomData<&'d T>,
    buffer_allocator: buffer::Allocator,
    ep_allocator: state::EndpointAllocator<'static, T>,
    //alloc: [EndpointData; EP_COUNT],
    //
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Speed {
    /// Throttle to low / full speeds.
    ///
    /// If a host is capable of high-speed, this will prevent
    /// the device from enumerating as a high-speed device.
    LowFull,
    /// High speed.
    ///
    /// A high-speed device can still interface a low / full
    /// speed host, so use this setting for the most flexibility.
    #[default]
    High,
}

impl<'d, T: Instance> Driver<'d, T> {
    pub fn new<const SIZE: usize, const N: usize>(
        buffer: &'static mut buffer::EndpointMemory<SIZE>,
        state: &'static mut state::EndpointState<T, N>,
    ) -> Self {
        let ep_allocator = state.allocator().expect("Endpoint state already assigned");
        let buffer_allocator = buffer
            .allocator()
            .expect("Endpoint memory already assigned");

        let mut res = Self {
            ep_allocator,
            buffer_allocator,
            _phantom: PhantomData,
        };

        res.init();

        T::ControllerInterrupt::unpend();
        unsafe { T::ControllerInterrupt::enable() };

        res
    }

    fn init(&mut self) {
        let phy = T::phy_regs();
        let usb = T::regs();
        phy.ctrl_set().write(|v| v.set_sftrst(true));
        phy.ctrl_clr().write(|v| v.set_sftrst(true));
        phy.ctrl_clr().write(|v| v.set_clkgate(true));
        phy.pwd().write_value(Pwd(0));

        usb.usbcmd().write(|w| w.set_rst(true));

        while usb.usbcmd().read().rst() {}

        usb.usbcmd()
            .write(|w| w.set_itc(pac::usb::vals::Itc::ITC_0));

        usb.usbmode().write(|v| {
            v.set_cm(pac::usb::vals::Cm::CM_2);
            v.set_slom(true);
        });

        let speed = Speed::High;
        usb.portsc1().modify(|v| {
            v.set_pfsc(speed == Speed::LowFull);
        });

        usb.usbsts().write_value(usb.usbsts().read());
        usb.usbintr().write_value(Usbintr(0));

        usb.asynclistaddr()
            .write_value(Asynclistaddr(self.ep_allocator.qh_list_addr() as u32));
    }

    pub fn allocate_buffer(&mut self, max_packet_len: usize) -> Option<buffer::Buffer> {
        self.buffer_allocator.allocate(max_packet_len)
    }

    pub fn is_allocated(&self, addr: EndpointAddress) -> bool {
        self.ep_allocator.endpoint(addr).is_some()
    }

    fn alloc_endpoint(
        &mut self,
        ep_dir: Direction,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<&'d mut Endpoint<T>, driver::EndpointAllocError> {
        if let Some(addr) = ep_addr {
            //if usb.is_allocated(addr) {
            //    return Err(usb_device::UsbError::InvalidEndpoint);
            //}
            let buffer = self
                .allocate_buffer(max_packet_size as usize)
                .ok_or(driver::EndpointAllocError {})?;

            let ep = unsafe {
                core::mem::transmute(
                    self.ep_allocator
                        .allocate_endpoint(addr, buffer, ep_type, interval_ms)
                        .unwrap(),
                )
            };

            Ok(ep)
        } else {
            for idx in 1..8 {
                let addr = EndpointAddress::from_parts(idx, ep_dir);
                if self.is_allocated(addr) {
                    continue;
                }
                let buffer = self
                    .allocate_buffer(max_packet_size as usize)
                    .ok_or(driver::EndpointAllocError)?;

                let ep = self
                    .ep_allocator
                    .allocate_endpoint(addr, buffer, ep_type, interval_ms)
                    .unwrap();

                return Ok(unsafe { core::mem::transmute(ep) });
            }

            Err(driver::EndpointAllocError {})
        }
    }

    /// Initialize (or reinitialize) all non-zero endpoints
    fn initialize_endpoints(&mut self) {
        for ep in self.ep_allocator.nonzero_endpoints_iter_mut() {
            ep.initialize();
        }
    }

    pub fn bus_reset(&mut self) {
        T::regs()
            .endptstat()
            .write_value(T::regs().endptstat().read());

        T::regs()
            .endptcomplete()
            .write_value(T::regs().endptcomplete().read());

        T::regs()
            .endptnak()
            .write_value(T::regs().endptnak().read());
        T::regs()
            .endptnak()
            .write_value(T::regs().endptnak().read());
        T::regs().endptnaken().write_value(Endptnaken(0));

        while T::regs().endptprime().read().0 != 0 {}

        T::regs().endptflush().write_value(Endptflush(u32::MAX));

        while T::regs().endptflush().read().0 != 0 {}

        debug_assert!(
            T::regs().portsc1().read().pr(),
            "Took too long to handle bus reset"
        );

        self.initialize_endpoints();
    }
}

pub struct ControlPipe<'d, T: Instance> {
    _phantom: PhantomData<T>,
    ep_out: &'d mut Endpoint<T>,
    ep_in: &'d mut Endpoint<T>,
}

impl<'d, T: Instance> ControlPipe<'d, T> {
    pub fn has_setup(&self) -> bool {
        T::regs().endptsetupstat().read().endptsetupstat() & 1 != 0
    }
}

impl<'d, T: Instance> driver::ControlPipe for ControlPipe<'d, T> {
    fn max_packet_size(&self) -> usize {
        self.ep_out.max_packet_len()
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|cx| {
            EP_OUT_WAKERS[0].register(cx.waker());

            if self.has_setup() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        let setup = self.ep_out.read_setup();

        if !self.ep_out.is_primed() {
            self.ep_out.clear_nack();
            let max_packet_len = self.ep_out.max_packet_len();
            self.ep_out.schedule_transfer(max_packet_len);
        }

        setup.to_le_bytes()
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        self.ep_out.read(buf).await
    }

    async fn data_in(
        &mut self,
        data: &[u8],
        _first: bool,
        _last: bool,
    ) -> Result<(), EndpointError> {
        self.ep_in.write(data).await
    }

    async fn accept(&mut self) {
        if self.ep_in.is_primed() {
            //trace!("EP IS PRIMED WHEN IT SHOULDNT BE");
        }

        self.ep_in.clear_nack();

        let written = Endpoint::write_buffer(self.ep_in, &[]);
        self.ep_in.schedule_transfer(written);

        if !self.ep_out.is_primed() {
            self.ep_out.clear_nack();
            self.ep_out.schedule_transfer(0);
        }

        poll_fn(|cx| {
            EP_IN_WAKERS[0].register(cx.waker());

            if T::regs().endptcomplete().read().etce() & 1 != 0 {
                T::regs().endptcomplete().write(|v| v.set_etce(1));

                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }

    async fn reject(&mut self) {
        self.ep_in.set_stalled(true);
        self.ep_out.set_stalled(true);
    }

    async fn accept_set_address(&mut self, addr: u8) {
        T::regs().deviceaddr().write(|v| {
            v.set_usbadr(addr);
            v.set_usbadra(true);
        });

        self.accept().await;
    }
}

impl<'d, T: Instance> driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = &'d mut Endpoint<T>;
    type EndpointIn = &'d mut Endpoint<T>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, driver::EndpointAllocError> {
        self.alloc_endpoint(Direction::In, None, ep_type, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, driver::EndpointAllocError> {
        self.alloc_endpoint(Direction::Out, None, ep_type, max_packet_size, interval_ms)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let ep_out = self
            .alloc_endpoint(
                Direction::Out,
                Some(EndpointAddress::from_parts(0, Direction::Out)),
                EndpointType::Control,
                control_max_packet_size,
                0,
            )
            .unwrap();

        let ep_in = self
            .alloc_endpoint(
                Direction::In,
                Some(EndpointAddress::from_parts(0, Direction::In)),
                EndpointType::Control,
                control_max_packet_size,
                0,
            )
            .unwrap();

        T::regs().usbintr().modify(|v| {
            v.set_ue(true);
            v.set_uee(true);
            v.set_ure(true);
            v.set_pce(true);
            v.set_see(true);
        });

        T::regs().usbcmd().modify(|v| v.set_rs(true));

        T::regs().usbsts().write_value(T::regs().usbsts().read());

        (
            Bus { driver: self },
            ControlPipe {
                _phantom: PhantomData,
                ep_out,
                ep_in,
            },
        )
    }
}

pub struct Bus<'d, T: Instance> {
    driver: Driver<'d, T>,
}
impl<'d, T: Instance> driver::Bus for Bus<'d, T> {
    async fn poll(&mut self) -> Event {
        poll_fn(move |cx| {
            BUS_WAKER.register(cx.waker());

            if IRQ_PORT_CHANGE.load(Ordering::Acquire) {
                IRQ_PORT_CHANGE.store(false, Ordering::Relaxed);
                let portsc1 = T::regs().portsc1().read();

                let ccs = T::regs().portsc1().read().ccs();
                if ccs {
                    return Poll::Ready(Event::PowerDetected);
                } else {
                    return Poll::Ready(Event::PowerRemoved);
                }
            }

            if IRQ_RESUME.load(Ordering::Acquire) {
                IRQ_RESUME.store(false, Ordering::Relaxed);
                return Poll::Ready(Event::Resume);
            }

            if IRQ_RESET.load(Ordering::Acquire) {
                self.driver.bus_reset();

                for w in &EP_IN_WAKERS {
                    w.wake();
                }
                for w in &EP_OUT_WAKERS {
                    w.wake();
                }

                IRQ_RESET.store(false, Ordering::Relaxed);

                return Poll::Ready(Event::Reset);
            }

            if IRQ_SUSPEND.load(Ordering::Acquire) {
                IRQ_SUSPEND.store(false, Ordering::Relaxed);
                return Poll::Ready(Event::Suspend);
            }

            Poll::Pending
        })
        .await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stall: bool) {
        let ep = self.driver.ep_allocator.endpoint_mut(ep_addr).unwrap();
        ep.set_stalled(stall);

        // Re-prime any OUT endpoints if we're unstalling
        if !stall && ep_addr.direction() == Direction::Out && !ep.is_primed() {
            let max_packet_len = ep.max_packet_len();
            ep.schedule_transfer(max_packet_len);
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let ep = self.driver.ep_allocator.endpoint_mut(ep_addr).unwrap();
        ep.is_stalled()
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let ep = self
            .driver
            .ep_allocator
            .endpoint_mut(ep_addr)
            .expect("endpoint does not exist");

        if enabled {
            ep.enable();

            if ep_addr.direction() == Direction::In {
                EP_IN_WAKERS[ep_addr.index()].wake();
            } else {
                if !ep.is_primed() {
                    ep.schedule_transfer(ep.max_packet_len());
                }
                EP_OUT_WAKERS[ep_addr.index()].wake();
            }
        } else {
            ep.disable();
            // TODO: Flush EP prime status
        }
    }

    async fn enable(&mut self) {}

    async fn disable(&mut self) {}

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

mod sealed {
    use super::*;

    pub trait Instance {
        type ControllerInterrupt: interrupt::typelevel::Interrupt;
        type PhyInterrupt: interrupt::typelevel::Interrupt;

        fn regs() -> pac::usb::Usb;
        fn phy_regs() -> pac::usbphy::Usbphy;
    }
}

pub trait Instance: sealed::Instance + 'static {}

macro_rules! impl_instance {
    ($controller_inst:ident, $controller_irq:ident, $phy_inst: ident, $phy_irq:ident) => {
        impl sealed::Instance for peripherals::$controller_inst {
            type ControllerInterrupt = crate::interrupt::typelevel::$controller_irq;
            type PhyInterrupt = crate::interrupt::typelevel::$phy_irq;

            fn regs() -> pac::usb::Usb {
                pac::$controller_inst
            }

            fn phy_regs() -> pac::usbphy::Usbphy {
                pac::$phy_inst
            }
        }
        impl Instance for peripherals::$controller_inst {}
    };
}

impl_instance!(USB, USB_OTG1, USBPHY, USB_PHY);
