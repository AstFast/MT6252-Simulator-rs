//! 中断控制器（0x8101_xxxx）。
//!
//! 硬件语义：向 CLR 寄存器写 1 表示"允许"该中断线，向 SET 寄存器写 1 表示"屏蔽"。
//! C 版用一个全局变量手工模拟这个位图，这里同样只维护位图，不碰假内存。

use crate::memmap::intc;

/// 无中断挂起时硬件返回的值。
///
/// **必须是 `0x100`（bit8），不能是 `0xffff`。** 派发器（flash `0x0800_2E38`）算的是
/// `index = [INT_STATUS] & 0x3F`，并单独测 bit8：bit8 置位就直接返回、不派发也不写 EOI。
/// 用 `0xffff` 的话 `& 0x13F = 0x3F`，等于**真的去派发 LISR 表的第 63 槽**。
pub const NOT_PENDING: u32 = 0x100;

#[derive(Debug)]
pub struct IrqCtrl {
    enabled_l: u32,
    enabled_h: u32,
    status: u32,
}

impl Default for IrqCtrl {
    fn default() -> Self {
        Self { enabled_l: 0, enabled_h: 0, status: NOT_PENDING }
    }
}

impl IrqCtrl {
    pub fn write(&mut self, addr: u32, value: u32) {
        match addr {
            intc::INT_MASK_CLR_L => self.enabled_l |= value,
            intc::INT_MASK_CLR_H => self.enabled_h |= value,
            intc::INT_MASK_SET_L => self.enabled_l &= !value,
            intc::INT_MASK_SET_H => self.enabled_h &= !value,
            intc::INT_STATUS => self.status = value,
            // 中断处理完写 EOI，状态回到"无挂起"
            intc::INT_EOI_L | intc::INT_EOI_H | intc::FIQ_FEOI => self.status = NOT_PENDING,
            // LISR 派发路径的 ack 走的是这个地址（不是 EOI_L），CTIRQ1 就靠它清状态
            intc::INT_ACK => self.status = NOT_PENDING,
            _ => {}
        }
    }

    pub fn read(&self, addr: u32) -> Option<u32> {
        match addr {
            // 掩码寄存器返回"被屏蔽"的位图，与硬件一致
            intc::INT_MASK_STA_L => Some(!self.enabled_l),
            intc::INT_MASK_STA_H => Some(!self.enabled_h),
            intc::INT_STATUS => Some(self.status),
            _ => None,
        }
    }

    /// 注入一条中断线，固件在 ISR 里读 `INT_STATUS` 取中断源。
    ///
    /// **参数是物理线号，不是固件 `IRQ_Register_LISR` 用的逻辑号。** 固件有三套编号：
    /// ROM `0x0850FA7C` 是 physical→logical 表，开机时 `0x08001B00` 把它反转成 RAM 里的
    /// logical→physical 表（`0→0, 1→2, 2→3, 3→4, 4→5, 5→6, 6→1, 7 以上恒等`），
    /// `IRQ_Register_LISR`（`0x0819F274`）用 `ldrb` 查这张表得到物理号，再拿它索引
    /// `0x4000C840` 上每项 12 字节的 LISR 表；`INT_STATUS[5:0]`、掩码位、EOI 位也全是物理号。
    /// 实测对得上：slot2 = CTIRQ1（逻辑 1）、slot3 = L1D_CTIRQ2（逻辑 2）、
    /// slot6 = DMA（逻辑 5）、slot1 = L1SM（逻辑 6，**唯一一条往回映射的**）。
    /// 逻辑→物理的换算由调用方 `Vm::raise_irq` 做，因为只有它拿得到 guest 内存里的表。
    pub fn trigger(&mut self, phys_line: u32) {
        self.status = phys_line;
    }

    /// 记录一次掩码变更，便于回答"固件到底开没开某条中断线"
    pub fn log_change(&self, addr: u32, value: u32) {
        match addr {
            intc::INT_MASK_CLR_L | intc::INT_MASK_CLR_H | intc::INT_MASK_SET_L | intc::INT_MASK_SET_H => {
                println!(
                    "[intc] {addr:#010x}={value:#010x} enabled_l={:#010x} enabled_h={:#010x}",
                    self.enabled_l, self.enabled_h
                );
            }
            _ => {}
        }
    }

    /// 低 32 条线的使码位图，只用于日志
    pub fn enabled_bits(&self) -> u32 {
        self.enabled_l
    }

    pub fn enabled(&self, line: u32) -> bool {
        let (word, bit) = if line < 32 { (self.enabled_l, line) } else { (self.enabled_h, line - 32) };
        (word >> bit) & 1 == 1
    }

    /// CPSR bit7：IRQ 被全局屏蔽
    pub fn cpu_masked(cpsr: u32) -> bool {
        cpsr & (1 << 7) != 0
    }
}
