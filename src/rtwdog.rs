use super::pac;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Clock {
    Bus = 0b00,
    Lpo = 0b01,
    Internal = 0b10,
    External = 0b11,
}

pub struct Config {
    clock: Clock,
    prescaler: bool,
    count: u16,
}

impl Config {
    pub const fn new(clock: Clock, prescaler: bool, count: u16) -> Self {
        Self {
            clock,
            prescaler,
            count,
        }
    }

    pub const fn lpo(interval: embassy_time::Duration) -> Self {
        let lpo_scaled = 128;
        let count = (interval.as_millis() * lpo_scaled) / 1000;

        Self {
            clock: Clock::Lpo,
            prescaler: true,
            count: count as _,
        }
    }
}

#[inline]
pub fn unlock() {
    pac::RTWDOG
        .cnt()
        .write_value(pac::rtwdog::regs::Cnt(0xD928C520));

    while !pac::RTWDOG.cs().read().ulk() {}
}

#[inline]
pub fn disable() {
    unlock();

    pac::RTWDOG.cs().modify(|r| r.set_en(false));
}

#[inline]
pub fn enable(config: Config) {
    unlock();

    pac::RTWDOG
        .toval()
        .write_value(pac::rtwdog::regs::Toval(config.count as _));

    pac::RTWDOG.cs().write(|r| {
        r.set_en(true);
        r.set_pres(config.prescaler);
        r.set_clk(config.clock as u8);
        r.set_win(false);
        r.set_update(true);
        r.set_cmd32en(true);
    });

    while !pac::RTWDOG.cs().read().rcs() {}
}

#[inline]
pub fn feed() {
    pac::RTWDOG
        .cnt()
        .write_value(pac::rtwdog::regs::Cnt(0xB480A602));
}

#[inline]
pub fn count() -> u16 {
    pac::RTWDOG.cnt().read().0 as u16
}
