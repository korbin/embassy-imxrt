use core::sync::atomic::AtomicUsize;

pub struct Ringbuffer<const SIZE: usize> {
    buffer: [u8; SIZE],

    read_ptr: AtomicUsize,
    write_ptr: AtomicUsize,
}

impl<const SIZE: usize> Ringbuffer<SIZE> {
    pub const fn new() -> Self {
        let buffer = [0; SIZE];

        Self {
            buffer,
            read_ptr: AtomicUsize::new(0),
            write_ptr: AtomicUsize::new(0),
        }
    }
}

impl<const SIZE: usize> Ringbuffer<SIZE> {
    pub fn push() {
        //
    }
}

pub struct Consumer<'a, const SIZE: usize> {
    ringbuffer: &'a mut Ringbuffer<SIZE>,
}
