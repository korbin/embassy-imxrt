#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum SerialClockFrequency {
    /// No change, keep current serial clock unchanged
    NoChange = 0,
    MHz30,
    MHz50,
    MHz60,
    #[cfg(not(feature = "imxrt1170"))]
    MHz75,
    MHz80,
    MHz100,
    #[cfg(any(feature = "imxrt1060", feature = "imxrt1064", feature = "imxrt1170"))]
    MHz120,
    MHz133,
    #[cfg(any(feature = "imxrt1060", feature = "imxrt1064"))]
    MHz166,
}

pub mod fields;
pub mod lookup;

pub use fields::*;
pub use lookup::*;

pub struct Config {}

impl Config {
    pub const fn new() -> Self {
        Self {}
    }
}

/// The recommended `csHoldTime`, `0x03`.
///
/// This is the default value if not set with [`ConfigurationBlock::cs_hold_time`].
pub const RECOMMENDED_CS_HOLD_TIME: u8 = 0x03;
/// The recommended `csSetupTime`, `0x03`.
///
/// This is the default value if not set with [`ConfigurationBlock::cs_setup_time`].
pub const RECOMMENDED_CS_SETUP_TIME: u8 = 0x03;

pub mod fcb {
    use super::fields::*;
    use super::lookup::LookupTable;

    pub const TAG: [u8; 4] = *b"FCFB";

    /// A version identifier.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[repr(transparent)]
    pub struct Version(u32);

    impl Version {
        /// Construct a version number for your FCB.
        ///
        /// Once constructed, pass the version to the configuration block with
        /// [`ConfigurationBlock::version`](ConfigurationBlock::version).
        pub const fn new(major: u8, minor: u8, bugfix: u8) -> Version {
            Version(
                ((b'V' as u32) << 24)
                    | ((major as u32) << 16)
                    | ((minor as u32) << 8)
                    | bugfix as u32,
            )
        }
    }

    impl Default for Version {
        fn default() -> Self {
            Self::new(1, 0, 0)
        }
    }

    #[derive(Debug, Clone, Copy)]
    #[repr(C, packed)]
    pub struct ConfigurationBlock {
        tag: u32,
        version: Version,

        _reserved0: [u8; 4], // 0x008
        read_sample_clk_src: ReadSampleClockSource,
        cs_hold_time: u8,
        cs_setup_time: u8,
        column_address_width: ColumnAddressWidth,
        device_mode_configuration: u8,
        /// TODO: this isn't reserved on 1170.
        /// It's "device mode type", with a default value
        /// of "generic."
        device_mode_type: u8, // 0x011
        wait_time_cfg_commands: WaitTimeConfigurationCommands,
        device_mode_sequence: DeviceModeSequence,
        device_mode_arg: u32,
        config_cmd_enable: u8,
        _reserved2: [u8; 3], // 0x01D
        config_cmd_seqs: [u8; 12],
        _reserved3: [u8; 4], // 0x02C
        cfg_cmd_args: [u8; 12],
        _reserved4: [u8; 4], // 0x03C
        controller_misc_options: u32,
        pub(crate) device_type: u8,
        serial_flash_pad_type: FlashPadType,
        serial_clk_freq: SerialClockFrequency,
        lut_custom_seq_enable: u8,
        _reserved5: [u8; 8], // 0x048
        /// A1, A2, B1, B2
        serial_flash_sizes: [u32; 4],
        cs_pad_setting_override: u32,
        sclk_pad_setting_override: u32,
        data_pad_setting_override: u32,
        dqs_pad_setting_override: u32,
        timeout_ms: u32,
        command_interval: u32,
        data_valid_time: u32,
        busy_offset: u16,
        busy_bit_polarity: u16,
        lookup_table: LookupTable,
        lut_custom_seq: [u8; 48],
        _reserved6: [u8; 16],
    }
}

//#[derive(Debug, Clone, Copy)]
//#[repr(C, packed)]
//pub struct ConfigurationBlock {
//    mem_cfg: flexspi::ConfigurationBlock,
//    page_size: u32,
//    sector_size: u32,
//    ip_cmd_serial_clk_freq: SerialClockFrequency,
//    is_uniform_block_size: u8,
//    is_data_order_swapped: u8,
//    _reserved0: [u8; 5],
//    block_size: u32,
//}
//
//impl ConfigurationBlock {
//    /// Create a new serial NOR configuration block based on the FlexSPI configuration
//    /// block
//    pub const fn new(mut mem_cfg: ConfigurationBlock) -> Self {
//        mem_cfg.device_type = 1;
//        ConfigurationBlock {
//            is_uniform_block_size: 0,
//            is_data_order_swapped: 0,
//            _reserved0: [0; 5],
//            block_size: 0,
//            mem_cfg,
//            page_size: 0,
//            sector_size: 0,
//            ip_cmd_serial_clk_freq: SerialClockFrequency::NoChange,
//        }
//    }
//    /// Set the serial NOR page size
//    pub const fn page_size(mut self, page_size: u32) -> Self {
//        self.page_size = page_size;
//        self
//    }
//    /// Set the serial NOR sector size
//    pub const fn sector_size(mut self, sector_size: u32) -> Self {
//        self.sector_size = sector_size;
//        self
//    }
//    /// Set the serial clock frequency
//    pub const fn ip_cmd_serial_clk_freq(
//        mut self,
//        serial_clock_frequency: SerialClockFrequency,
//    ) -> Self {
//        self.ip_cmd_serial_clk_freq = serial_clock_frequency;
//        self
//    }
//}
//
//impl ConfigurationBlock {
//    /// Set the serial NOR block size if it differs from the sector size.
//    ///
//    /// By default, the configuration block signals to the hardware that the
//    /// sector size is the same as the block size. Calling this will override
//    /// that setting, allowing you to configure a different block size.
//    ///
//    /// The behavior is unspecified if you call this with a block size that's
//    /// equal to the sector size.
//    pub const fn block_size(mut self, block_size: u32) -> Self {
//        self.is_uniform_block_size = 0u8;
//        self.block_size = block_size;
//        self
//    }
//}
//
//const _STATIC_ASSERT_SIZE: [u32; 1] =
//    [0; (core::mem::size_of::<ConfigurationBlock>() == 512) as usize];
//
//#[cfg(test)]
//mod test {
//    use super::{flexspi, ConfigurationBlock, SerialClockFrequency};
//    use crate::flexspi::LookupTable;
//
//    #[test]
//    fn smoke() {
//        const _CFG: ConfigurationBlock =
//            ConfigurationBlock::new(flexspi::ConfigurationBlock::new(LookupTable::new()))
//                .page_size(256)
//                .sector_size(4095)
//                .ip_cmd_serial_clk_freq(SerialClockFrequency::MHz30);
//    }
//}
