#![allow(clippy::declare_interior_mutable_const)] // Usage is legit in this module.

use core::{
    cell::UnsafeCell,
    mem::MaybeUninit,
    sync::atomic::{AtomicU32, Ordering},
};

use embassy_usb_driver::{Direction, EndpointAddress, EndpointType};

use super::{buffer::Buffer, qh::Qh, td::Td, Endpoint, Instance};

/// A list of transfer descriptors
///
/// Supports 1 TD per QH (per endpoint direction)
#[repr(align(32))]
struct TdList<const COUNT: usize>([UnsafeCell<Td>; COUNT]);

impl<const COUNT: usize> TdList<COUNT> {
    const fn new() -> Self {
        const TD: UnsafeCell<Td> = UnsafeCell::new(Td::new());
        Self([TD; COUNT])
    }
}

/// A list of queue heads
///
/// One queue head per endpoint, per direction (default).
#[repr(align(4096))]
struct QhList<const COUNT: usize>([UnsafeCell<Qh>; COUNT]);

impl<const COUNT: usize> QhList<COUNT> {
    const fn new() -> Self {
        const QH: UnsafeCell<Qh> = UnsafeCell::new(Qh::new());
        Self([QH; COUNT])
    }
}

/// The collection of endpoints.
///
/// Maintained inside the EndpointState so that it's sized just right.
struct EpList<T: Instance, const COUNT: usize>([UnsafeCell<MaybeUninit<Endpoint<T>>>; COUNT]);

impl<T: Instance, const COUNT: usize> EpList<T, COUNT> {
    const EP: UnsafeCell<MaybeUninit<Endpoint<T>>> = UnsafeCell::new(MaybeUninit::uninit());
    const fn new() -> Self {
        Self([Self::EP; COUNT])
    }
}

/// The maximum supported number of endpoints.
///
/// Eight endpoints, two in each direction. Any endpoints allocated
/// beyond this are wasted.
pub const MAX_ENDPOINTS: usize = 8 * 2;

/// Produces an index into the EPs, QHs, and TDs collections
fn index(ep_addr: EndpointAddress) -> usize {
    (ep_addr.index() * 2) + (Direction::In == ep_addr.direction()) as usize
}

/// Driver state associated with endpoints.
///
/// Each USB driver needs an `EndpointState`. Allocate a `static` object
/// and supply it to your USB constructor. Make sure that states are not
/// shared across USB instances; otherwise, the driver constructor panics.
///
/// Use [`max_endpoints()`](EndpointState::max_endpoints) if you're not interested in reducing the
/// memory used by this allocation. The default object holds enough
/// state for all supported endpoints.
///
/// ```
/// use embassy_imxrt::usb::EndpointState;
///
/// static EP_STATE: EndpointState = EndpointState::max_endpoints();
/// ```
///
/// If you know that you can use fewer endpoints, you can control the
/// memory utilization with the const generic `COUNT`. You're expected
/// to provide at least two endpoints -- one in each direction -- for
/// control endpoints.
///
/// Know that endpoints are allocated in pairs; all even endpoints are
/// OUT, and all odd endpoints are IN. For example, a `COUNT` of 5 will
/// have 3 out endpoints, and 2 in endpoints. You can never have more
/// IN that OUT endpoints without overallocating OUT endpoints.
///
/// ```
/// use embassy_imxrt::usb::EndpointState;
///
/// static EP_STATE: EndpointState<5> = EndpointState::new();
/// ```
///
/// Any endpoint state allocated beyond [`MAX_ENDPOINTS`] are wasted.
pub struct EndpointState<T: Instance, const COUNT: usize = MAX_ENDPOINTS> {
    qh_list: QhList<COUNT>,
    td_list: TdList<COUNT>,
    ep_list: EpList<T, COUNT>,
    /// Low 16 bits are used for tracking endpoint allocation.
    /// Bit 31 is set when the allocator is first taken. This
    /// bit is always dropped during u32 -> u16 conversions.
    alloc_mask: AtomicU32,
}

unsafe impl<T: Instance, const COUNT: usize> Sync for EndpointState<T, COUNT> {}

impl<T: Instance> EndpointState<T, MAX_ENDPOINTS> {
    /// Allocate space for the maximum number of endpoints.
    ///
    /// Use this if you don't want to consider the exact number
    /// of endpoints that you might need.
    pub const fn max_endpoints() -> Self {
        Self::new()
    }
}

impl<T: Instance, const COUNT: usize> EndpointState<T, COUNT> {
    /// Allocate state for `COUNT` endpoints.
    pub const fn new() -> Self {
        Self {
            qh_list: QhList::new(),
            td_list: TdList::new(),
            ep_list: EpList::new(),
            alloc_mask: AtomicU32::new(0),
        }
    }

    /// Acquire the allocator.
    ///
    /// Returns `None` if the allocator was already taken.
    pub fn allocator(&self) -> Option<EndpointAllocator<T>> {
        const ALLOCATOR_TAKEN: u32 = 1 << 31;
        let alloc_mask = self.alloc_mask.fetch_or(ALLOCATOR_TAKEN, Ordering::SeqCst);
        (alloc_mask & ALLOCATOR_TAKEN == 0).then(|| EndpointAllocator {
            qh_list: &self.qh_list.0[..self.qh_list.0.len().min(MAX_ENDPOINTS)],
            td_list: &self.td_list.0[..self.td_list.0.len().min(MAX_ENDPOINTS)],
            ep_list: &self.ep_list.0[..self.ep_list.0.len().min(MAX_ENDPOINTS)],
            alloc_mask: &self.alloc_mask,
        })
    }
}

pub struct EndpointAllocator<'a, T: Instance> {
    qh_list: &'a [UnsafeCell<Qh>],
    td_list: &'a [UnsafeCell<Td>],
    ep_list: &'a [UnsafeCell<MaybeUninit<Endpoint<T>>>],
    alloc_mask: &'a AtomicU32,
}

unsafe impl<T: Instance> Send for EndpointAllocator<'_, T> {}

impl<T: Instance> EndpointAllocator<'_, T> {
    /// Atomically inserts the endpoint bit into the allocation mask, returning `None` if the
    /// bit was already set.
    fn try_mask_update(&mut self, mask: u16) -> Option<()> {
        let mask = mask.into();
        (mask & self.alloc_mask.fetch_or(mask, Ordering::SeqCst) == 0).then_some(())
    }

    /// Returns `Some` if the endpoint is allocated.
    fn check_allocated(&self, index: usize) -> Option<()> {
        (index < self.qh_list.len()).then_some(())?;
        let mask = 1u16 << index;
        (mask & self.alloc_mask.load(Ordering::SeqCst) as u16 != 0).then_some(())
    }

    /// Acquire the QH list address.
    ///
    /// Used to tell the hardware where the queue heads are located.
    pub fn qh_list_addr(&self) -> *const () {
        self.qh_list.as_ptr().cast()
    }

    /// Acquire the endpoint.
    ///
    /// Returns `None` if the endpoint isn't allocated.
    pub fn endpoint(&self, addr: EndpointAddress) -> Option<&Endpoint<T>> {
        let index = index(addr);
        self.check_allocated(index)?;

        // Safety: there's no other mutable access at this call site.
        // Perceived lifetime is tied to the EndpointAllocator, which has an
        // immutable receiver.

        let ep = unsafe { &*self.ep_list[index].get() };
        // Safety: endpoint is allocated. Checked above.
        Some(unsafe { ep.assume_init_ref() })
    }

    /// Implementation detail to permit endpoint iteration.
    ///
    /// # Safety
    ///
    /// This can only be called from a method that takes a mutable receiver.
    /// Otherwise, you could reach the same mutable endpoint more than once.
    unsafe fn endpoint_mut_inner(&self, addr: EndpointAddress) -> Option<&mut Endpoint<T>> {
        let index = index(addr);
        self.check_allocated(index)?;

        // Safety: the caller ensures that we actually have a mutable reference.
        // Once we have a mutable reference, this is equivalent to calling the
        // safe UnsafeCell::get_mut method.
        let ep = unsafe { &mut *self.ep_list[index].get() };

        // Safety: endpoint is allocated. Checked above.
        Some(unsafe { ep.assume_init_mut() })
    }

    /// Aquire the mutable endpoint.
    ///
    /// Returns `None` if the endpoint isn't allocated.
    pub fn endpoint_mut(&mut self, addr: EndpointAddress) -> Option<&mut Endpoint<T>> {
        // Safety: call from method with mutable receiver.
        unsafe { self.endpoint_mut_inner(addr) }
    }

    /// Return an iterator of all allocated endpoints.
    pub fn endpoints_iter_mut(&mut self) -> impl Iterator<Item = &mut Endpoint<T>> {
        (0..8)
            .flat_map(|index| {
                let ep_out = EndpointAddress::from_parts(index, Direction::Out);
                let ep_in = EndpointAddress::from_parts(index, Direction::In);
                [ep_out, ep_in]
            })
            // Safety: call from method with mutable receiver.
            .flat_map(|ep| unsafe { self.endpoint_mut_inner(ep) })
    }

    /// Returns an iterator for all non-zero, allocated endpoints.
    ///
    /// "Non-zero" excludes the first two control endpoints.
    pub fn nonzero_endpoints_iter_mut(&mut self) -> impl Iterator<Item = &mut Endpoint<T>> {
        self.endpoints_iter_mut()
            .filter(|ep| ep.address().index() != 0)
    }

    /// Allocate the endpoint for the specified address.
    ///
    /// Returns `None` if any are true:
    ///
    /// - The endpoint is already allocated.
    /// - We cannot allocate an endpoint for the given address.
    pub fn allocate_endpoint(
        &mut self,
        addr: EndpointAddress,
        buffer: Buffer,
        kind: EndpointType,
        interval_ms: u8,
    ) -> Option<&mut Endpoint<T>> {
        let index = index(addr);
        (index < self.qh_list.len()).then_some(())?;
        let mask = 1u16 << index;

        // If we pass this call, we're the only caller able to observe mutable
        // QHs, TDs, and EPs at index.
        self.try_mask_update(mask)?;

        // Safety: index in range. Atomic update on alloc_mask prevents races for
        // allocation, and ensures that we only release one &mut reference for each
        // component.
        let qh = unsafe { &mut *self.qh_list[index].get() };
        let td = unsafe { &mut *self.td_list[index].get() };
        // We cannot access these two components after this call. The endpoint
        // takes mutable references, so it has exclusive ownership of both.
        // This module is designed to isolate this access so we can visually
        // see where we have these &mut accesses.

        // EP is uninitialized.
        let ep = unsafe { &mut *self.ep_list[index].get() };
        // Nothing to drop here.
        ep.write(Endpoint::new(addr, qh, td, buffer, kind, interval_ms));
        // Safety: EP is initialized.
        Some(unsafe { ep.assume_init_mut() })
    }
}
