//! MT6252 DMA 控制器中被我们用到的三个通道：MSDC、SIM1、SIM2。
//!
//! 通道寄存器的语义：先写 control/addr/count，再向 start 写 `0x8000` 表示"启动传输"。
//! 我们把它翻译成"配置就绪"，由具体设备（SD 卡、SIM 卡）在需要数据的那一刻完成搬运。

use crate::memmap::dma_reg;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// 内存 → 外设
    RamToReg,
    /// 外设 → 内存
    RegToRam,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Align {
    Byte,
    Halfword,
    Word,
}

#[derive(Debug, Clone)]
pub struct Channel {
    pub control: u32,
    /// 传输的字节数（已按 align 换算过）
    pub transfer_count: u32,
    pub align: Align,
    pub direction: Direction,
    pub ch: u32,
    /// 数据缓冲区在目标内存中的地址
    pub data_addr: u32,
    /// start 写过 0x8000，配置就绪待搬运
    pub configured: bool,
    pub int_enable: bool,
}

impl Default for Channel {
    fn default() -> Self {
        Self {
            control: 0,
            transfer_count: 0,
            align: Align::Byte,
            direction: Direction::RamToReg,
            ch: 0,
            data_addr: 0,
            configured: false,
            int_enable: false,
        }
    }
}

impl Channel {
    fn apply_control(&mut self, value: u32) {
        self.control = value;
        self.ch = (value >> 20) & 0b1_1111;
        self.direction = if (value >> 18) & 1 == 0 { Direction::RamToReg } else { Direction::RegToRam };
        self.align = match value & 0b11 {
            2 => Align::Word,
            1 => Align::Halfword,
            _ => Align::Byte,
        };
        self.int_enable = (value >> 15) & 1 == 1;
    }

    fn apply_count(&mut self, value: u32) {
        self.transfer_count = match self.align {
            Align::Word => value * 4,
            Align::Halfword => value * 2,
            Align::Byte => value,
        };
    }
}

#[derive(Debug, Default)]
pub struct Dma {
    pub msdc: Channel,
    pub sim1: Channel,
    pub sim2: Channel,
}

/// 通道块大小：每个通道占 0x100
const CHANNEL_BLOCK: u32 = dma_reg::CHANNEL_BLOCK;

impl Dma {
    /// 把绝对地址映射到 (通道, 通道内偏移)
    fn resolve(&mut self, addr: u32) -> Option<(&mut Channel, u32)> {
        let block = addr.checked_sub(dma_reg::BASE)? / CHANNEL_BLOCK;
        let off = addr.wrapping_sub(dma_reg::BASE) % CHANNEL_BLOCK;
        let ch = match block {
            b if b == dma_reg::MSDC_CHANNEL / CHANNEL_BLOCK => &mut self.msdc,
            b if b == dma_reg::SIM1_CHANNEL / CHANNEL_BLOCK => &mut self.sim1,
            b if b == dma_reg::SIM2_CHANNEL / CHANNEL_BLOCK => &mut self.sim2,
            _ => return None,
        };
        Some((ch, off))
    }

    pub fn write(&mut self, addr: u32, value: u32) {
        let Some((ch, off)) = self.resolve(addr) else { return };
        match off {
            dma_reg::OFF_TRANSFER_COUNT => ch.apply_count(value),
            dma_reg::OFF_CONTROL => ch.apply_control(value),
            dma_reg::OFF_START if value == 0x8000 => {
                ch.configured = true;
                println!(
                    "[dma] 通道启动 ch={:#x} dir={:?} count={} addr={:#x}",
                    ch.ch, ch.direction, ch.transfer_count, ch.data_addr
                );
            }
            dma_reg::OFF_DATA_ADDR => ch.data_addr = value,
            _ => {}
        }
    }

    pub fn channel_mut(&mut self, which: ChannelId) -> &mut Channel {
        match which {
            ChannelId::Msdc => &mut self.msdc,
            ChannelId::Sim1 => &mut self.sim1,
            ChannelId::Sim2 => &mut self.sim2,
        }
    }

    pub fn channel(&self, which: ChannelId) -> &Channel {
        match which {
            ChannelId::Msdc => &self.msdc,
            ChannelId::Sim1 => &self.sim1,
            ChannelId::Sim2 => &self.sim2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelId {
    Msdc,
    Sim1,
    Sim2,
}
