use bitvec::prelude::*;
use core::{
    future::poll_fn,
    marker::PhantomData,
    sync::atomic::{compiler_fence, Ordering},
    task::Poll,
};
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver as driver;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType,
};

use super::Instance;

use crate::pac;

use super::{
    buffer::Buffer,
    qh::Qh,
    td::{Status, Td},
};

pub trait Dir {
    fn dir() -> Direction;
    fn waker(i: usize) -> &'static AtomicWaker;
}

pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }

    #[inline]
    fn waker(i: usize) -> &'static AtomicWaker {
        &super::EP_IN_WAKERS[i]
    }
}

pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }

    #[inline]
    fn waker(i: usize) -> &'static AtomicWaker {
        &super::EP_OUT_WAKERS[i]
    }
}

pub struct Endpoint<T: Instance> {
    _phantom: PhantomData<T>,
    qh: &'static mut Qh,
    td: &'static mut Td,
    buffer: Buffer,
    pub info: EndpointInfo,
}

impl<T: Instance> Endpoint<T> {
    pub fn new(
        address: EndpointAddress,
        qh: &'static mut Qh,
        td: &'static mut Td,
        buffer: Buffer,
        kind: EndpointType,
        interval_ms: u8,
    ) -> Self {
        let max_packet_size = buffer.len();
        qh.set_zero_length_termination(false);
        qh.set_max_packet_len(max_packet_size);
        qh.set_interrupt_on_setup(
            EndpointType::Control == kind && address.direction() == Direction::Out,
        );

        td.set_terminate();
        td.clear_status();

        let info = EndpointInfo {
            addr: address,
            max_packet_size: max_packet_size as u16,
            ep_type: kind,
            interval_ms,
        };

        Endpoint {
            _phantom: PhantomData,
            qh,
            td,
            buffer,
            info,
        }
    }

    ///////// Enable ZLT for the given endpoint.
    //////pub fn enable_zlt(&mut self) {
    //////    self.qh.set_zero_length_termination(true);
    //////}

    /// Indicates if the transfer descriptor is active
    pub fn is_primed(&self) -> bool {
        (match self.info.addr.direction() {
            Direction::In => T::regs().endptstat().read().etbr(),
            Direction::Out => T::regs().endptstat().read().erbr(),
        } & (1 << self.info.addr.index()))
            != 0
    }

    /// Check for any transfer status, which is signaled through
    /// an error
    pub fn check_errors(&self) -> Result<(), embassy_usb_driver::EndpointError> {
        let status = self.td.status();
        if status.contains(Status::TRANSACTION_ERROR)
            | status.contains(Status::DATA_BUFFER_ERROR)
            | status.contains(Status::HALTED)
        {
            Err(embassy_usb_driver::EndpointError::Disabled)
        } else {
            Ok(())
        }
    }

    /// Initialize the endpoint, should be called soon after it's assigned,
    /// or after transitioning out of configuration (reset the endpoint).
    pub fn initialize(&mut self) {
        if self.info.addr.index() != 0 {
            let endptctrl = T::regs().endptctrl_x(self.info.addr.index());

            match self.info.addr.direction() {
                Direction::In => {
                    endptctrl.modify(|v| {
                        v.set_txe(false);
                        v.set_txt(EndpointType::Bulk as u8);
                    });
                }
                Direction::Out => {
                    endptctrl.modify(|v| {
                        v.set_rxe(false);
                        v.set_rxt(EndpointType::Bulk as u8);
                    });
                }
            }
        }
    }

    /// Returns the endpoint address
    pub fn address(&self) -> EndpointAddress {
        self.info.addr
    }

    /// Returns the maximum packet length supported by this endpoint
    pub fn max_packet_len(&self) -> usize {
        self.qh.max_packet_len()
    }

    /// Read the setup buffer from this endpoint
    /// This is only meaningful for a control OUT endpoint.
    pub fn read_setup(&mut self) -> u64 {
        // Reference manual isn't really clear on whe we should clear the ENDPTSETUPSTAT
        // bit...
        //
        // - section "Control Endpoint Operational Model" says that we should clear it
        //   *before* attempting to read the setup buffer, but
        // - section "Operational Model For Setup Transfers" says to do it *after*
        //   we read the setup buffer
        //
        // We're going with the "before" approach here. (Reference manual is iMXRT1060, rev2)
        loop {
            T::regs().usbcmd().modify(|v| v.set_sutw(true));

            let setup = self.qh.setup();
            if T::regs().usbcmd().read().sutw() {
                T::regs().usbcmd().modify(|v| v.set_sutw(false));

                T::regs()
                    .endptsetupstat()
                    .write(|v| v.set_endptsetupstat(1 << self.info.addr.index()));
                return setup;
            }
        }
    }

    /// Read data from the endpoint into `buffer`
    ///
    /// Returns the number of bytes read into `buffer`, which is constrained by the
    /// max packet length, and the number of bytes received in the last transfer.
    pub fn read_buffer(&mut self, buffer: &mut [u8]) -> usize {
        let size = self
            .qh
            .max_packet_len()
            .min(buffer.len())
            .min(self.td.bytes_transferred());
        self.buffer.volatile_read(&mut buffer[..size])
    }

    /// Write `buffer` to the endpoint buffer
    ///
    /// Returns the number of bytes written from `buffer`, which is constrained
    /// by the max packet length.
    pub fn write_buffer(&mut self, buffer: &[u8]) -> usize {
        let size = self.qh.max_packet_len().min(buffer.len());
        let written = self.buffer.volatile_write(&buffer[..size]);
        self.buffer.clean_invalidate_dcache(size);
        written
    }

    /// Schedule a transfer of `size` bytes from the endpoint buffer
    ///
    /// Caller should check to see if there is an active transfer, or if the previous
    /// transfer resulted in an error or halt.
    pub fn schedule_transfer(&mut self, size: usize) {
        self.td.set_terminate();
        self.td.set_buffer(self.buffer.as_ptr_mut(), size);
        self.td.set_interrupt_on_complete(true);
        self.td.set_active();
        self.td.clean_invalidate_dcache();

        self.qh.overlay_mut().set_next(self.td);
        self.qh.overlay_mut().clear_status();
        self.qh.clean_invalidate_dcache();

        match self.info.addr.direction() {
            Direction::In => {
                T::regs()
                    .endptprime()
                    .write(|v| v.set_petb(1 << self.info.addr.index()));
            }
            Direction::Out => {
                T::regs()
                    .endptprime()
                    .write(|v| v.set_perb(1 << self.info.addr.index()));
            }
        }
        while T::regs().endptprime().read().0 != 0 {}
    }

    /// Stall or unstall the endpoint
    pub fn set_stalled(&mut self, stall: bool) {
        let endptctrl = T::regs().endptctrl_x(self.info.addr.index());

        match self.info.addr.direction() {
            Direction::In => {
                endptctrl.modify(|v| v.set_txs(stall));
            }
            Direction::Out => {
                endptctrl.modify(|v| v.set_rxs(stall));
            }
        }
    }

    /// Indicates if the endpoint is stalled
    pub fn is_stalled(&self) -> bool {
        let endptctrl = T::regs().endptctrl_x(self.info.addr.index());

        match self.info.addr.direction() {
            Direction::In => endptctrl.read().txs(),
            Direction::Out => endptctrl.read().rxs(),
        }
    }

    /// Enable the endpoint
    ///
    /// This should be called only after the USB device has been configured.
    pub fn enable(&mut self) {
        // EP0 is always enabled
        if self.info.addr.index() != 0 {
            let endptctrl = T::regs().endptctrl_x(self.info.addr.index());

            match self.info.addr.direction() {
                Direction::In => {
                    endptctrl.modify(|v| {
                        v.set_txe(true);
                        v.set_txr(true);
                        v.set_txt(self.info.ep_type as u8);
                    });
                    super::EP_IN_WAKERS[self.info.addr.index()].wake();
                }
                Direction::Out => {
                    endptctrl.modify(|v| {
                        v.set_rxe(true);
                        v.set_rxr(true);
                        v.set_rxt(self.info.ep_type as u8);
                    });
                    super::EP_OUT_WAKERS[self.info.addr.index()].wake();
                }
            }
        }
    }

    pub fn disable(&mut self) {
        // EP0 is always enabled
        if self.info.addr.index() != 0 {
            let endptctrl = T::regs().endptctrl_x(self.info.addr.index());
            match self.info.addr.direction() {
                Direction::In => {
                    super::EP_IN_WAKERS[self.info.addr.index()].wake();
                    endptctrl.modify(|v| {
                        v.set_txe(true);
                        v.set_txr(true);
                        v.set_txt(self.info.ep_type as u8);
                    });
                }
                Direction::Out => {
                    super::EP_OUT_WAKERS[self.info.addr.index()].wake();

                    endptctrl.modify(|v| {
                        v.set_rxe(false);
                        v.set_rxr(false);
                        v.set_rxt(self.info.ep_type as u8);
                    });
                }
            }
        }
    }

    /// Indicates if this endpoint is enabled
    ///
    /// Endpoint 0, the control endpoint, is always enabled.
    pub fn is_enabled(&self) -> bool {
        if self.info.addr.index() == 0 {
            return true;
        }

        let endptctrl = T::regs().endptctrl_x(self.info.addr.index());
        match self.info.addr.direction() {
            Direction::In => endptctrl.read().txe(),
            Direction::Out => endptctrl.read().rxe(),
        }
    }

    /// Clear the NACK bit for this endpoint
    pub fn clear_nack(&mut self) {
        match self.info.addr.direction() {
            Direction::In => {
                T::regs()
                    .endptnak()
                    .write(|v| v.set_eptn(1 << self.info.addr.index()));
            }
            Direction::Out => {
                T::regs()
                    .endptnak()
                    .write(|v| v.set_eprn(1 << self.info.addr.index()));
            }
        }
    }
}

impl<T: Instance> driver::Endpoint for &mut Endpoint<T> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let index = self.info.addr.index();
        let dir = self.address().direction();

        poll_fn(|cx| {
            if dir == Direction::In {
                super::EP_IN_WAKERS[index].register(cx.waker());
            } else {
                super::EP_OUT_WAKERS[index].register(cx.waker());
            }

            if self.is_enabled() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }
}

impl<T: Instance> driver::EndpointOut for &mut Endpoint<T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        self.check_errors()?;

        let index = self.info.addr.index();

        poll_fn(|cx| {
            super::EP_OUT_WAKERS[index].register(cx.waker());

            if !self.is_enabled() {
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            let primed = self.is_primed();
            let completed = T::regs().endptcomplete().read().erce() & 1 << index != 0;
            if completed && !primed {
                T::regs().endptcomplete().write(|v| v.set_erce(1 << index));

                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
        .await?;

        self.clear_nack();

        let read = Endpoint::read_buffer(self, buf);

        self.schedule_transfer(self.max_packet_len());

        Ok(read)
    }
}

impl<T: Instance> driver::EndpointIn for &mut Endpoint<T> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        self.check_errors()?;

        let index = self.info.addr.index();

        self.clear_nack();

        let written = Endpoint::write_buffer(self, buf);

        self.schedule_transfer(written);

        poll_fn(|cx| {
            if !self.is_enabled() {
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            super::EP_IN_WAKERS[index].register(cx.waker());
            let completed = T::regs().endptcomplete().read().etce() & 1 << index != 0;
            if completed {
                T::regs().endptcomplete().write(|v| v.set_etce(1 << index));

                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}
