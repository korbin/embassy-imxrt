//! Timer driver.

use core::cell::{Cell, RefCell};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::interrupt::InterruptExt;
use crate::{interrupt, pac};

struct AlarmState {
    timestamp: Cell<u64>,
}
unsafe impl Send for AlarmState {}

struct PITDriver {
    alarms: Mutex<CriticalSectionRawMutex, AlarmState>,
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

impl Driver for PITDriver {
    fn now(&self) -> u64 {
        loop {
            let hi = pac::PIT.ltmr64h().read();
            let lo = pac::PIT.ltmr64l().read();
            let hi2 = pac::PIT.ltmr64h().read();

            if hi == hi2 {
                return u64::MAX - ((hi as u64) << 32 | (lo as u64));
            }
        }
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: PITDriver = PITDriver {
    alarms:  Mutex::const_new(CriticalSectionRawMutex::new(), AlarmState {
        timestamp: Cell::new(0),
    }),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

impl PITDriver {
    unsafe fn init(&'static self) {
        pac::CCM.ccgr1().modify(|r| r.set_cg6(0b00));

        pac::CCM.cscmr1().modify(|r| {
            r.set_perclk_podf(pac::ccm::vals::PerclkPodf::DIVIDE_24);
            r.set_perclk_clk_sel(true);
        });

        pac::CCM.ccgr1().modify(|r| r.set_cg6(0b11));

        pac::PIT.mcr().write(|w| w.set_mdis(true));

        pac::PIT
            .timer(0)
            .tctrl()
            .write_value(pac::pit::regs::Tctrl(0));
        pac::PIT
            .timer(1)
            .tctrl()
            .write_value(pac::pit::regs::Tctrl(0));
        pac::PIT
            .timer(2)
            .tctrl()
            .write_value(pac::pit::regs::Tctrl(0));
        pac::PIT
            .timer(3)
            .tctrl()
            .write_value(pac::pit::regs::Tctrl(0));

        pac::PIT.timer(0).ldval().write_value(u32::MAX);
        pac::PIT.timer(1).ldval().write_value(u32::MAX);
        pac::PIT.timer(2).ldval().write_value(0);
        pac::PIT.timer(3).ldval().write_value(0);

        pac::PIT.mcr().modify(|w| w.set_mdis(false));

        pac::PIT.timer(1).tctrl().write(|v| {
            v.set_chn(true);
            v.set_ten(true)
        });
        pac::PIT.timer(0).tctrl().write(|v| v.set_ten(true));

        crate::interrupt::PIT.enable();

        pac::PIT.timer(2).tctrl().write(|v| v.set_tie(true));
        pac::PIT.timer(3).tctrl().write(|v| v.set_tie(true));
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let n = 0;
        let alarm = &self.alarms.borrow(cs);
        alarm.timestamp.set(timestamp);

        let timer = pac::PIT.timer(2 + n);

        timer.tctrl().modify(|x| x.set_ten(false));
        timer.tflg().modify(|x| x.set_tif(true));

        let now = self.now();
        if timestamp <= now {
            alarm.timestamp.set(u64::MAX);

            return false;
        }

        timer.ldval().write_value((timestamp - now) as u32);
        timer.tctrl().modify(|x| x.set_ten(true));

        true
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self
            .queue
            .borrow(cs)
            .borrow_mut()
            .next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self
                .queue
                .borrow(cs)
                .borrow_mut()
                .next_expiration(self.now());
        }
    }

    fn check_alarm(&self) {
        critical_section::with(|cs| {
            let n = 0;
            let timer = pac::PIT.timer(2 + n);

            let alarm = &self.alarms.borrow(cs);

            let interrupted = timer.tflg().read().tif();
            timer.tflg().write(|r| r.set_tif(true));

            if interrupted {
                timer.tctrl().modify(|r| r.set_ten(false));

                let now = self.now();
                let timestamp = alarm.timestamp.get();

                if timestamp <= now {
                    self.trigger_alarm(cs);
                } else {
                    timer.ldval().write_value((timestamp - now) as u32);
                    timer.tctrl().modify(|r| r.set_ten(true));
                }
            }
        });
    }
}

#[cfg(feature = "rt")]
#[no_mangle]
pub extern "C" fn PIT() {
    DRIVER.check_alarm();
}

pub fn init() {
    unsafe {
        DRIVER.init();
    }
}
