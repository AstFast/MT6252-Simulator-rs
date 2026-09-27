//! SIM 卡控制器（0x8109 / 0x810f）。
//!
//! ISO7816 T=0/T=1 的时序被压缩成"状态机 + 中断"：固件写 TIDE 触发一次传输，
//! 我们通过 COUNT/RX 中断把 ATR 或响应字节一个一个喂回去。
//! 骨架阶段只实现 ATR 复位与 SELECT/GET RESPONSE 两条命令，其余命令记录后返回 SW=0x6D25。

use crate::engine::Engine;
use crate::memmap::sim_reg::{self, BASE1, BASE2};

/// 复位应答（TS=0x3B，T0=0x00 表示无历史字节）
const ATR: &[u8] = &[0x3b, 0x00, 1, 2, 3, 4, 5, 6];
const APDU_SELECT: &[u8] = &[0xa0, 0xa4, 0x00, 0x00, 0x02];
const APDU_GET_RESPONSE: &[u8] = &[0xa0, 0xc0, 0x00, 0x00, 0x16];
/// select DF_GSM 的数据体，以及它的响应
const DATA_SELECT_DF_GSM: &[u8] = &[0x7f, 0x20];
const RSP_SF_7F20: &[u8] = &[
    0xA0, 0xC0, 0x32, 0x32, 0x0F, 0x32, 0x32, 0x32, 0x08, 0x2F, 0x05, 0x04, 0x34, 0x01, 0xFF, 0x55, 0x01, 0x02, 0x32,
    0x32, 0x90, 0x32,
];

#[allow(dead_code)] // 硬件位定义镜像
pub mod irq {
    pub const TX: u32 = 1;
    pub const RX: u32 = 2;
    pub const TOUT: u32 = 8;
    pub const NOATR: u32 = 0x20;
    pub const T0END: u32 = 0x80;
    pub const RXERR: u32 = 0x100;
    pub const T1END: u32 = 0x200;
    pub const EDCERR: u32 = 0x400;
    pub const DMA_CMD: u32 = 0x800;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// 上电后等待固件写 TIDE 开始复位
    Idle,
    /// ATR 之后的历史字节（当前 ATR 无历史字节，保留给真卡响应）
    AtrHistory,
    /// 正常命令阶段
    Command,
}

#[derive(Debug)]
pub struct SimCard {
    pub index: u8,
    phase: Phase,
    pub irq_enable: u32,
    pub irq_status: u32,
    /// 待送出的字节
    pending: Vec<u8>,
    /// 固件逐字节写进 DATA 寄存器的 APDU
    apdu: Vec<u8>,
    /// 固件通过 DMA 送进来的命令数据体（例如文件 ID）
    cmd_data: Vec<u8>,
    pub irq_channel: u32,
    pub irq_pending: bool,
    count: u32,
    sw1: u32,
    sw2: u32,
    control: u32,
    /// 未建模寄存器的**影子**：C 版把整段 `0x8109_0000` 当普通 RAM 映射，所以固件对
    /// `+0x04/+0x08/+0x20/+0x24/+0x48` 这些没在 switch 里出现的偏移做"读-改-写"时，写进去的
    /// 值能原样读回来。我们这边是 MMIO 设备，不接这一支就会把写丢掉、读回 0，
    /// 固件的 `ldr/orr/str` 序列于是每次都在清它刚置上的位。
    shadow: [u32; 32],
    /// 上一次 T0 命令需要回传的字节数
    t0_response: usize,
}

impl SimCard {
    pub fn new(index: u8) -> Self {
        Self {
            index,
            phase: Phase::Idle,
            irq_enable: 0,
            irq_status: 0,
            pending: Vec::new(),
            apdu: Vec::new(),
            cmd_data: Vec::new(),
            irq_channel: 0,
            irq_pending: false,
            count: 0,
            sw1: 0,
            sw2: 0,
            control: 0,
            shadow: [0; 32],
            t0_response: 0,
        }
    }

    fn base(&self) -> u32 {
        if self.index == 0 { BASE1 } else { BASE2 }
    }

    pub fn read(&mut self, addr: u32) -> Option<u32> {
        let off = addr - self.base();
        match off {
            sim_reg::OFF_CARD_TYPE => Some(0x20), // 传统卡
            sim_reg::OFF_DATA => {
                // 从待送队列头部吐一个字节，剩余长度由 COUNT 读出时反映。
                // **排空之后再读是正常情况**：我们按整份 ATR（8 字节）记账，而驱动那次 TIDE 只要
                // 2 字节，它随后还会自己回来取残留。这里不能无条件 `remove(0)` —— 空队列上它会
                // panic 掉整个仿真线程（实测四次跑全死在片 2780 上下同一处）。
                let byte = if self.pending.is_empty() { 0 } else { self.pending.remove(0) };
                self.count = self.count.saturating_sub(1);
                Some(byte as u32)
            }
            sim_reg::OFF_COUNT => Some(self.count),
            sim_reg::OFF_SW1 => Some(self.sw1),
            sim_reg::OFF_SW2 => Some(self.sw2),
            sim_reg::OFF_IRQ_STATUS => Some(self.irq_status),
            sim_reg::OFF_CONTROL => Some(self.control),
            // 其余偏移按 C 版"整段当普通 RAM"的等价行为读回固件自己写进去的值
            _ => self.shadow.get((off / 4) as usize).copied(),
        }
    }

    pub fn write(&mut self, addr: u32, value: u32) {
        let off = addr - self.base();
        // 先把值落到影子里：C 版这些位置是真 RAM，写进去的一定读得回来；
        // 下面各分支的语义处理是"额外"效果，不取代影子。
        if let Some(s) = self.shadow.get_mut((off / 4) as usize) {
            *s = value;
        }
        match off {
            sim_reg::OFF_TIDE => self.start_transfer(value),
            sim_reg::OFF_IRQ_ENABLE => {
                self.irq_enable = value;
                print!("[sim{}] 中断使能", self.index);
                for (bit, name) in [
                    (irq::TX, "TX"),
                    (irq::RX, "RX"),
                    (irq::TOUT, "TOUT"),
                    (irq::NOATR, "NOATR"),
                    (irq::RXERR, "RXERR"),
                    (irq::T0END, "T0END"),
                    (irq::T1END, "T1END"),
                ] {
                    if value & bit != 0 {
                        print!(" [{name}]");
                    }
                }
                println!(" ({value:#x})");
            }
            sim_reg::OFF_CONTROL => self.control = value,
            // 中断状态：硬件置位、驱动清位。C 版这段是普通 RAM，驱动的写天然生效；
            // 我们是 MMIO 设备，不接这一支的话驱动清 ISR 位的写会被丢掉。
            sim_reg::OFF_IRQ_STATUS => self.irq_status = value,
            sim_reg::OFF_DATA => {
                self.apdu.push(value as u8);
                println!("[sim{}] 收到命令字节 {value:#04x}", self.index);
            }
            sim_reg::OFF_COUNT => self.count = value,
            _ => {}
        }
    }

    /// TIDE：低 16 位是 RX 触发长度，高 16 位是 TX 触发长度
    fn start_transfer(&mut self, value: u32) {
        let rx_len = (value & 0xf) + 1;
        let tx_len = ((value >> 16) & 0xf) + 1;
        println!("[sim{}] TIDE rx={rx_len} tx={tx_len} phase={:?}", self.index, self.phase);
        match self.phase {
            Phase::Idle => {
                self.pending = ATR.to_vec();
                self.count = self.pending.len() as u32;
                self.phase = if ATR[1] & 0x80 != 0 { Phase::AtrHistory } else { Phase::Command };
                self.irq_channel = irq::RX;
                self.irq_pending = true;
            }
            Phase::AtrHistory => {
                self.irq_channel = irq::RX;
                self.irq_pending = true;
            }
            Phase::Command => {}
        }
    }

    /// 固件通过 DMA 把命令数据体送进来了，结合 DATA 寄存器攒下的 APDU 一起解析
    pub fn on_tx_finished(&mut self, eng: Engine, dma_addr: u32, len: u32) {
        self.cmd_data = eng.read_bytes(dma_addr, len as usize);
        println!("[sim{}] APDU: {:02x?} data: {:02x?}", self.index, self.apdu, self.cmd_data);
        if self.apdu == APDU_SELECT {
            println!("[sim{}] select file", self.index);
            self.sw1 = 0x9f;
            self.sw2 = RSP_SF_7F20.len() as u32;
            self.t0_response = RSP_SF_7F20.len();
            self.irq_channel = irq::T0END;
            self.irq_pending = true;
        } else if self.apdu == APDU_GET_RESPONSE {
            self.sw1 = 0x90;
            self.sw2 = 0x00;
        } else {
            println!("[sim{}] 未实现的 APDU，返回 6D25", self.index);
            self.sw1 = 0x6d;
            self.sw2 = 0x25;
        }
        self.apdu.clear();
    }

    /// 固件开启 DMA 接收，把响应数据交给它
    pub fn on_rx_finished(&mut self, eng: Engine, dma_addr: u32, len: u32) {
        if self.t0_response == 0 {
            println!("[sim{}] 无待响应数据", self.index);
            return;
        }
        if self.cmd_data == DATA_SELECT_DF_GSM {
            println!("[sim{}] select df.gsm → 回 {} 字节", self.index, RSP_SF_7F20.len());
            let n = RSP_SF_7F20.len().min(len as usize);
            eng.write_bytes(dma_addr, &RSP_SF_7F20[..n]);
            self.sw1 = 0x90;
            self.sw2 = 0;
            self.t0_response = 0;
            self.irq_channel = irq::T0END;
            self.irq_pending = true;
        } else {
            println!("[sim{}] 未响应数据命令 {:02x?}", self.index, self.cmd_data);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 复位握手：TIDE 只要求 2 字节（TS+T0），但设备按整份 ATR 记账，固件随后还会自己回来
    /// 把残留取完 —— 所以"读到的字节数 > 当初要求的字节数"是正常轨迹，不是错误。
    #[test]
    fn atr_survives_a_short_request_and_the_drain_that_follows() {
        let mut card = SimCard::new(0);
        card.write(BASE1 + sim_reg::OFF_TIDE, 1); // rx_len = (1 & 0xf) + 1 = 2
        let got: Vec<u32> = (0..ATR.len())
            .map(|_| card.read(BASE1 + sim_reg::OFF_DATA).unwrap())
            .collect();
        let want: Vec<u32> = ATR.iter().map(|&b| b as u32).collect();
        assert_eq!(got, want);
    }

    /// 排空之后继续读必须回 0 而不是打崩仿真线程：`remove(0)` 在空队列上会 panic，
    /// 实测四次开机全部死在同一处（片 2780 上下），后面所有"跑不到更深处"的结论其实
    /// 量的都是这个崩溃点。
    #[test]
    fn reading_data_past_the_end_yields_zero_and_never_panics() {
        let mut card = SimCard::new(0);
        card.write(BASE1 + sim_reg::OFF_TIDE, 1);
        for _ in 0..ATR.len() {
            card.read(BASE1 + sim_reg::OFF_DATA);
        }
        for _ in 0..16 {
            assert_eq!(card.read(BASE1 + sim_reg::OFF_DATA), Some(0));
        }
        // COUNT 跟着饱和在 0，不会绕回大值让驱动的等待循环再等一轮
        assert_eq!(card.read(BASE1 + sim_reg::OFF_COUNT), Some(0));
    }
}
