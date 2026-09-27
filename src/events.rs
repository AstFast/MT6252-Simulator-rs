//! 仿真线程与 UI 线程之间的唯一通信通道。
//!
//! C 版用一个 256 槽位的静态数组 + mutex 手写队列（满了静默丢事件、出队整体搬移），
//! 这里换成 `std::sync::mpsc`：入队无锁竞争，也不会丢事件。

#[derive(Debug, Clone, Copy)]
pub enum VmEvent {
    /// 键盘矩阵：`key` 是 MT6252 键码，`down` 表示按下/松开
    Keyboard { key: u8, down: bool },
    /// RTC 秒中断（中断线 14）
    Rtc,
}

pub type EventSender = std::sync::mpsc::Sender<VmEvent>;
pub type EventReceiver = std::sync::mpsc::Receiver<VmEvent>;

pub fn channel() -> (EventSender, EventReceiver) {
    std::sync::mpsc::channel()
}

/// 固件的中断线编号，与 `docs/中断入口对照表.txt`、C 版 `StartInterrupt(n, ...)` 对齐
pub mod irq_line {
    /// 下面全是固件 `IRQ_Register_LISR` 用的**逻辑**号；注入时由 `Vm::phys_line` 查
    /// 固件自己建的 RAM 表（画像 `[interrupt] line_map`）换成物理号。
    ///
    /// C1XX 13MHz 定时器：`IRQ_Register_LISR(1, 0x400082F9, "CTIRQ1")`，物理号 2。
    pub const CTIRQ1_13M: u32 = 1;
    pub const KEYPAD: u32 = 8;
    pub const RTC_SEC: u32 = 14;
    /// USIM 卡 1：固件注册/EOI/掩码一律传 **4**（`IRQ_Register_LISR(4,…,"USIM_Lisr")`），
    /// 物理号才是 5。之前这里写的 5，等于查表后又多挪了一位，正好落到 DMA 的槽上。
    pub const SIM1: u32 = 4;
    pub const SIM2: u32 = 28;
}
