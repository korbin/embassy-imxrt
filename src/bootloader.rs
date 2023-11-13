use core::ffi::{c_char, CStr};

const BASE_ADDRESS: usize = 0x0020001c;

#[repr(C)]
struct BootloaderTree {
    version: u32,
    copyright: *const c_char,
    run_bootloader: extern "C" fn(*const u32),
}

pub fn version() -> u32 {
    unsafe {
        let base = (BASE_ADDRESS as *const *const BootloaderTree)
            .read_volatile()
            .read_volatile();
        base.version
    }
}

pub fn copyright() -> &'static CStr {
    unsafe {
        let base = (BASE_ADDRESS as *const *const BootloaderTree)
            .read_volatile()
            .read_volatile();

        CStr::from_ptr(base.copyright)
    }
}

pub fn run_bootloader(boot_mode: bool, image: bool) -> ! {
    unsafe {
        let base = (BASE_ADDRESS as *const *const BootloaderTree)
            .read_volatile()
            .read_volatile();

        let mut boot_arg: u32 = 0;
        boot_arg |= 0xEB << 24;
        boot_arg |= (boot_mode as u32) << 20; // boot mode; 0 = BMODE IN SMBR2 or fuse, 1 = serial downloader
        boot_arg |= 0x00 << 16; // serial downloader media ( 3 bits)
        boot_arg |= image as u32; // boot image; 0 = primary, 1 = redundant
        (base.run_bootloader)(&boot_arg)
    }

    loop {}
}

pub fn boot_dfu() -> ! {
    run_bootloader(true, false)
}

pub fn boot_fuses(primary: bool) -> ! {
    run_bootloader(false, primary)
}
