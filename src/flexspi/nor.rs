use super::{Flexspi, Instance};
use crate::flexspi::lookup::Command;
use crate::pac;
use embedded_storage::nor_flash::{
    check_erase, check_read, check_write, ErrorType, MultiwriteNorFlash, NorFlashError,
    NorFlashErrorKind, ReadNorFlash,
};

#[cfg(feature = "defmt")]
use defmt::*;
use pac::flexspi::regs::Dllcr;

pub const PAGE_SIZE: usize = 256;
/// Flash write size.
pub const WRITE_SIZE: usize = 256;
/// Flash read size.
pub const READ_SIZE: usize = 256;
/// Flash erase size.
pub const ERASE_SIZE: usize = 4096;

pub mod generic {
    use super::super::config::{self, *};
    use opcodes::sdr::*;

    pub const SEQ_READ: Sequence = SequenceBuilder::new()
        .instr(Instr::new(CMD, Pads::One, 0xEB))
        .instr(Instr::new(RADDR, Pads::Four, 0x18))
        .instr(Instr::new(DUMMY, Pads::Four, 0x06))
        .instr(Instr::new(READ, Pads::Four, 0x04))
        .build();
    pub const SEQ_READ_STATUS: Sequence = SequenceBuilder::new()
        .instr(Instr::new(CMD, Pads::One, 0x05))
        .instr(Instr::new(READ, Pads::One, 0x01))
        .build();
    pub const SEQ_WRITE_ENABLE: Sequence = SequenceBuilder::new()
        .instr(Instr::new(CMD, Pads::One, 0x06))
        .build();
    pub const SEQ_ERASE_SECTOR: Sequence = SequenceBuilder::new()
        .instr(Instr::new(CMD, Pads::One, 0x20))
        .instr(Instr::new(RADDR, Pads::One, 0x18))
        .build();
    pub const SEQ_READ_ID: Sequence = SequenceBuilder::new()
        .instr(Instr::new(CMD, Pads::One, 0x9f))
        .instr(Instr::new(READ, Pads::One, 0x01))
        .build();
    pub const SEQ_READ_SFDP: Sequence = SequenceBuilder::new()
        .instr(Instr::new(CMD, Pads::One, 0x5a))
        .instr(Instr::new(RADDR, Pads::One, 0x18))
        .instr(Instr::new(DUMMY, Pads::One, 0x08))
        .instr(Instr::new(READ, Pads::One, 0xff))
        .build();
    pub const SEQ_PAGE_PROGRAM: Sequence = SequenceBuilder::new()
        .instr(Instr::new(CMD, Pads::One, 0x02))
        .instr(Instr::new(RADDR, Pads::One, 0x18))
        .instr(Instr::new(WRITE, Pads::One, 0x04))
        .build();
    pub const SEQ_CHIP_ERASE: Sequence = SequenceBuilder::new()
        .instr(Instr::new(CMD, Pads::One, 0x60))
        .build();

    pub const LUT: LookupTable = LookupTable::new()
        .command(Command::Read, SEQ_READ)
        .command(Command::ReadStatus, SEQ_READ_STATUS)
        .command(Command::ReadId, SEQ_READ_ID)
        .command(Command::ReadSfdp, SEQ_READ_SFDP)
        .command(Command::WriteEnable, SEQ_WRITE_ENABLE)
        .command(Command::EraseSector, SEQ_ERASE_SECTOR)
        .command(Command::PageProgram, SEQ_PAGE_PROGRAM)
        .command(Command::ChipErase, SEQ_CHIP_ERASE);
}

pub struct NorFlash<'a, T: Instance, const FLASH_SIZE: usize> {
    flexspi: Flexspi<'a, T>,
}

impl<'a, T: Instance, const FLASH_SIZE: usize> NorFlash<'a, T, FLASH_SIZE> {
    pub fn from_flexspi(mut flexspi: Flexspi<'a, T>) -> Self {
        let p = T::regs();

        flexspi.disable();

        // Configure MCR0 with all initialization parameters
        p.mcr0().modify(|r| {
            r.set_rxclksrc(pac::flexspi::vals::Rxclksrc::RXCLKSRC_0);
            r.set_mdis(true);
            r.set_hsen(false);
            r.set_dozeen(false); // Enable doze mode like NXP SDK
            r.set_ahbgrantwait(0xFF);
            r.set_ipgrantwait(0xFF);
            r.set_combinationen(false);
            r.set_sckfreerunen(false);
            r.set_serclkdiv(pac::flexspi::vals::Serclkdiv::SERCLKDIV_0);
            r.set_ardfen(false);
            r.set_atdfen(false);
        });

        // Configure MCR1 - sequence and AHB bus wait
        p.mcr1().modify(|r| {
            r.set_seqwait(0xffff);
            r.set_ahbbuswait(0xffff);
        });

        // Configure MCR2 with proper settings
        p.mcr2().modify(|r| {
            r.set_resumewait(0x20);
            r.set_sckbdiffopt(false);
            r.set_samedeviceen(false);
            r.set_clrahbbufopt(false);
        });

        // Configure AHB control - critical for proper operation
        p.ahbcr().modify(|r| {
            r.set_prefetchen(false); // Disable prefetch initially
            r.set_cachableen(false); // Disable cache initially
            r.set_bufferableen(false); // Disable buffer initially
            r.set_readaddropt(true); // Enable read address optimization
        });

        // Configure all AHB RX buffers to match bootrom configuration
        for i in 0..4 {
            let buf = p.ahbrxbufcr0(i);
            buf.modify(|v| {
                v.set_prefetchen(true);
                v.set_priority(0);
                v.set_mstrid(i as u8); // Each buffer gets its own master ID (0,1,2,3)
                v.set_bufsz(32 / 8); // 32 bytes like bootrom (0x20 / 8 = 4)
            });
        }

        // Configure IP RX/TX FIFOs
        p.iprxfcr().modify(|r| {
            r.set_rxwmrk(0);
            r.set_rxdmaen(false); // Ensure DMA is disabled
        });

        p.iptxfcr().modify(|r| {
            r.set_txwmrk(0);
            r.set_txdmaen(false); // Ensure DMA is disabled
        });

        // Configure flash size
        p.flsha1cr0().modify(|v| {
            v.set_flshsz(2 * 1024); // Size in KB
        });

        // Configure write mask options
        p.flshcr4().modify(|r| {
            r.set_wmopt1(true);
            r.set_wmena(false); // Disable write mask for port A
            r.set_wmenb(false); // Disable write mask for port B
        });

        // Configure DLL - disabled for now (0x100 = OVRDEN bit set, DLLEN bit clear)
        p.dllcr(0).write_value(Dllcr(0x100));
        p.dllcr(1).write_value(Dllcr(0x100));

        // Configure flash control registers for each port
        for i in 0..1 {
            p.flshcr1(i).write(|r| {
                r.set_tcss(3);
                r.set_tcsh(3);
                r.set_csinterval(0);
                r.set_csintervalunit(false);
                r.set_cas(0);
                r.set_wa(false); // Word addressable mode
            });

            p.flshcr2(i).write(|r| {
                r.set_ardseqid(super::lookup::Command::Read as u8);
                r.set_ardseqnum(0); // Single sequence
                r.set_awrseqid(super::lookup::Command::PageProgram as u8);
                r.set_awrseqnum(0); // Single sequence
                r.set_awrwait(0);
                r.set_awrwaitunit(pac::flexspi::vals::Awrwaitunit::AWRWAITUNIT_0);
            });
        }

        flexspi.enable();

        cortex_m::asm::delay(100);

        flexspi.update_lut_entry(
            Command::ReadSfdp as u8,
            generic::LUT.sequence(Command::ReadSfdp),
        );

        flexspi.update_lut_entry(
            Command::ChipErase as u8,
            generic::LUT.sequence(Command::ChipErase),
        );
        flexspi.update_lut_entry(
            Command::PageProgram as u8,
            generic::LUT.sequence(Command::PageProgram),
        );
        flexspi.update_lut_entry(
            Command::ReadId as u8,
            generic::LUT.sequence(Command::ReadId),
        );
        flexspi.update_lut_entry(
            Command::EraseSector as u8,
            generic::LUT.sequence(Command::EraseSector),
        );
        flexspi.update_lut_entry(
            Command::WriteEnable as u8,
            generic::LUT.sequence(Command::WriteEnable),
        );
        flexspi.update_lut_entry(
            Command::ReadStatus as u8,
            generic::LUT.sequence(Command::ReadStatus),
        );
        flexspi.update_lut_entry(Command::Read as u8, generic::LUT.sequence(Command::Read));

        flexspi.reset();

        //p.intr().modify(|reg| {
        //    reg.set_ipcmderr(true);
        //    reg.set_ahbcmdge(true);
        //    reg.set_ipcmdge(true);
        //});

        Self { flexspi }
    }

    pub fn blocking_read(&mut self, mut address: u32, mut buf: &mut [u8]) -> Result<(), Error> {
        let p = T::regs();

        while !buf.is_empty() {
            let len = buf.len().min(128);
            let watermark = len.div_ceil(8).saturating_sub(1);
            p.iprxfcr().modify(|r| r.set_rxwmrk(watermark as _));

            self.cmd_start(false, address, len, Command::Read as u8, 0);

            let words_remaining = len.div_ceil(4);
            let entries_remaining = words_remaining.div_ceil(2);

            while (p.iprxfsts().read().fill() * 2) < entries_remaining as u8 {}

            for i in 0..words_remaining {
                let width = buf.len().min(4);
                let word = p.rfdr(i as _).read();
                let word = word.to_le_bytes();

                buf[..width].clone_from_slice(&word[..width]);
                buf = &mut buf[width..];
                address += width as u32;
            }

            p.intr().write(|r| r.set_iprxwa(true));

            self.flexspi.blocking_wait_bus_idle();
        }

        self.flexspi.reset_fifos(false, true);

        self.flexspi.blocking_wait_bus_idle();

        Ok(())
    }

    pub fn blocking_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        let end_offset = offset as usize + bytes.len();

        let padded_offset = (offset as *const u8).align_offset(PAGE_SIZE);
        let start_padding = core::cmp::min(padded_offset, bytes.len());

        if start_padding > 0 {
            let start = PAGE_SIZE - padded_offset;
            let end = start + start_padding;

            let mut pad_buf = [0xFF_u8; PAGE_SIZE];
            pad_buf[start..end].copy_from_slice(&bytes[..start_padding]);

            let unaligned_offset = offset as usize - start;

            self.blocking_page_program(unaligned_offset as _, &pad_buf);
        }

        let remaining_len = bytes.len() - start_padding;
        let end_padding = start_padding + PAGE_SIZE * (remaining_len / PAGE_SIZE);

        if remaining_len >= PAGE_SIZE {
            let mut aligned_offset = if start_padding > 0 {
                offset as usize + padded_offset
            } else {
                offset as usize
            };

            for chunk in bytes[start_padding..end_padding].chunks_exact(PAGE_SIZE) {
                let mut ram_buf = [0xFF_u8; PAGE_SIZE];
                ram_buf.copy_from_slice(chunk);

                self.blocking_page_program(aligned_offset as _, &ram_buf);

                aligned_offset += PAGE_SIZE;
            }
        }

        // Pad the end
        let rem_offset = (end_offset as *const u8).align_offset(PAGE_SIZE);
        let rem_padding = remaining_len % PAGE_SIZE;
        if rem_padding > 0 {
            let mut pad_buf = [0xFF_u8; PAGE_SIZE];
            pad_buf[..rem_padding].copy_from_slice(&bytes[end_padding..]);

            let unaligned_offset = end_offset - (PAGE_SIZE - rem_offset);

            self.blocking_page_program(unaligned_offset as _, &pad_buf);
        }

        Ok(())
    }

    fn cmd_start(&mut self, write: bool, address: u32, size: usize, seq_id: u8, seq_num: u8) {
        let p = T::regs();

        p.flshcr2(0).modify(|reg| {
            reg.set_clrinstrptr(true);
        });

        p.intr().modify(|reg| {
            reg.set_ahbcmderr(true);
            reg.set_ipcmderr(true);
            reg.set_ahbcmdge(true);
            reg.set_ipcmdge(true);
        });

        p.ipcr0().write_value(address);

        self.flexspi.reset_fifos(write, !write);

        p.ipcr0().write_value(address);

        p.ipcr1().write(|r| {
            r.set_idatsz(size as _);
            r.set_iseqid(seq_id);
            r.set_iseqnum(seq_num);
        });

        p.ipcmd().modify(|r| r.set_trg(true));
    }

    pub fn blocking_vendor_id(&mut self) -> u8 {
        let p = T::regs();

        self.cmd_start(false, 0x00, 1, Command::ReadId as u8, 0);
        self.flexspi.blocking_wait_bus_idle();
        let res = p.rfdr(0).read() as u8;
        self.flexspi.reset_fifos(false, true);

        res
    }

    pub fn blocking_read_sfdp(&mut self) -> u32 {
        let p = T::regs();

        self.cmd_start(false, 0x00, 4, Command::ReadSfdp as u8, 0);
        self.flexspi.blocking_wait_bus_idle();

        let res = p.rfdr(0).read();

        self.flexspi.reset_fifos(false, true);

        res
    }

    pub fn blocking_erase_chip(&mut self) {
        self.blocking_write_enable();
        self.cmd_start(true, 0, 0, Command::ChipErase as u8, 0);

        self.blocking_wait_write_busy();
        self.flexspi.blocking_wait_bus_idle();
    }

    pub fn blocking_erase_sector(&mut self, address: u32) {
        self.blocking_write_enable();
        self.cmd_start(true, address, 0, Command::EraseSector as u8, 0);

        self.flexspi.blocking_wait_bus_idle();
        self.blocking_wait_write_busy();
        self.flexspi.reset();
    }

    pub fn blocking_page_program(&mut self, mut address: u32, buf: &[u8; 256]) {
        let p = T::regs();
        let mut buf: &[u8] = &buf[..];

        while !buf.is_empty() {
            self.blocking_write_enable();

            let len = buf.len().min(128);
            let watermark = len.div_ceil(8).saturating_sub(1);

            self.cmd_start(true, address, len, Command::PageProgram as u8, 0);

            p.iptxfcr().modify(|r| r.set_txwmrk(watermark as _));
            while !p.intr().read().iptxwe() {}

            for (i, word) in buf[..len].chunks(4).enumerate() {
                let mut word_bytes: [u8; 4] = [0; 4];
                word_bytes[..word.len()].copy_from_slice(word);

                p.tfdr(i).write_value(u32::from_le_bytes(word_bytes));
            }

            buf = &buf[len..];
            address += len as u32;

            p.intr().modify(|r| r.set_iptxwe(true));

            self.flexspi.blocking_wait_bus_idle();

            self.blocking_wait_write_busy();

            cortex_m::asm::delay(1000);
        }

        self.flexspi.reset_fifos(true, false);

        self.flexspi.blocking_wait_bus_idle();
    }

    pub fn blocking_write_enable(&mut self) {
        let p = T::regs();

        self.cmd_start(true, 0, 0, Command::WriteEnable as u8, 0);

        // Wait for command done
        while !p.intr().read().ipcmddone() {}
        p.intr().modify(|r| r.set_ipcmddone(true));

        self.flexspi.blocking_wait_bus_idle();
    }

    pub fn blocking_enable_quad_mode(&mut self) {
        self.cmd_start(true, 0, 4, Command::ReadStatus as u8, 0);

        let p = T::regs();
        p.tfdr(0).write_value(0x40);

        self.flexspi.blocking_wait_bus_idle();
    }

    pub fn blocking_read_status(&mut self) -> u8 {
        let p = T::regs();

        self.cmd_start(false, 0, 1, Command::ReadStatus as u8, 0);

        self.flexspi.blocking_wait_bus_idle();

        let status = p.rfdr(0).read() as u8;

        self.flexspi.reset_fifos(false, true);

        status
    }

    pub fn blocking_write_busy(&mut self) -> bool {
        self.blocking_read_status() & 1 != 0
    }

    pub fn blocking_wait_write_busy(&mut self) {
        while self.blocking_write_busy() {
            cortex_m::asm::delay(100);
        }
    }
}

#[derive(Debug)]
pub struct Error {}

impl<'d, T: Instance, const FLASH_SIZE: usize> ErrorType for NorFlash<'d, T, FLASH_SIZE> {
    type Error = Error;
}

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

impl<'d, T: Instance, const FLASH_SIZE: usize> ReadNorFlash for NorFlash<'d, T, FLASH_SIZE> {
    const READ_SIZE: usize = READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl<'d, T: Instance, const FLASH_SIZE: usize> MultiwriteNorFlash for NorFlash<'d, T, FLASH_SIZE> {}

impl<'d, T: Instance, const FLASH_SIZE: usize> embedded_storage::nor_flash::NorFlash
    for NorFlash<'d, T, FLASH_SIZE>
{
    const WRITE_SIZE: usize = WRITE_SIZE;

    const ERASE_SIZE: usize = ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::check_erase(&self, from, to).map_err(|_| Error {})?;

        for address in
            (from..to).step_by(<Self as embedded_storage::nor_flash::NorFlash>::ERASE_SIZE)
        {
            self.blocking_erase_sector(address);
        }

        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(offset, bytes)
    }
}
