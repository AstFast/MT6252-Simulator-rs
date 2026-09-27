//! L1 协处理器在 `0x8001_xxxx` 信箱窗口里的寄存器（见 [`l1_win`]）。
//!
//! L1D 驱动每拍都会跑一串"读出来不等于常量就什么都不做"的检查（运行时 `0x4000822c`），
//! 其中几处比较的是 L1 侧才有的字。用画像的 `read_override` 伪装也能过，但那样就把
//! "设备行为"降级成"常量表"，L1 真正要参与握手时没有落脚点。
//!
//! 命令握手（运行时 `0x40008DC4`）是有往返的，所以状态由本设备持有：
//!
//! ```text
//! 等 STATUS.busy == 0 → REQUEST.go = 1 → 等 STATUS.accept == 1
//!   → COMMAND = 0x3181, COMMAND = 0x3080 → 等 RESPONSE.ready、RESPONSE.done
//!   → REQUEST.go = 0 → 返回 0（成功）
//! ```
//!
//! ROM 里没有任何一行代码能置起 `STATUS.accept` 或 `RESPONSE` 的那两位 —— 只有 L1
//! 这个总线主设备能写，所以这一步必须由模型做。反过来说，`RESPONSE` 两位的**取值**
//! 是从等待它们的指令读出来的，L1 内部到底算了什么没有建模。

use crate::memmap::l1_win;

#[derive(Debug, Default)]
pub struct L1Mailbox {
    /// [`l1_win::STATUS`]：bit15 忙（恒为 0 = 不忙）、bit0 已接受请求
    status: u32,
    /// [`l1_win::RESPONSE`]：命令被取走后置起 ready|done
    response: u32,
    /// 已完成的握手笔数。只打第一笔：L1D 拿着同一个应答反复重试（实测 30 秒内三万笔），
    /// 全打就是十几万行日志
    transactions: u64,
    /// 读 [`l1_win::COMMAND`] 的 (调用者 PC, 次数)。用来把"到底谁在读这个寄存器"问清楚 ——
    /// 静态扫字面量扫不到它：地址是算出来的（`0x80010020 + 0xE0 + 0x18`），
    /// 而手写指令解码器又因为 `LDRH/STRH` T1 的 imm5 要乘 2 而给出错偏移。
    /// 只报每个新 PC 的第一次，最多 `READ_TRACES` 条。
    readers: Vec<(u32, u64)>,
}

/// 每个不同的调用者 PC 只报一次，最多报这么多条（避免自旋时刷屏）
const READ_TRACES: usize = 16;

impl L1Mailbox {
    /// 命令字此刻该让 AP 读到的值。**第一次握手完成之前是 `0x3703`，之后是 0** ——
    /// 原来这条门控写在 `read()` 里，信箱页改成真内存之后它变成模型状态的唯一出口，
    /// 由写钩子按边沿把值推进页里（见 `Vm::l1_page_write`）。
    pub fn command(&self) -> u32 {
        if self.transactions == 0 { l1_win::COMMAND_ANSWER } else { 0 }
    }
    pub fn status(&self) -> u32 {
        self.status
    }
    pub fn response(&self) -> u32 {
        self.response
    }

    pub fn read(&mut self, addr: u32, pc: u32) -> Option<u32> {
        match addr {
            l1_win::STATE => Some(l1_win::STATE_VALUE),
            l1_win::READY_A => Some(l1_win::READY_A_VALUE),
            l1_win::READY_B => Some(l1_win::READY_B_VALUE),
            // 恒返回 0x3703 和"事务没开着就返回 0"都实测过，都不行：
            // L1D 每拍链在**举 REQUEST.go 之前**就有一道门要求 `(u16)[此] & ~0x80 == 0x3703`
            // （ROM `0x0870_3E60`，过了它才会 `bl 0x0870_49C0` 去做握手），所以一开始就回 0
            // 会让握手一次都开不起来，深度冻在和基线一模一样的地方。
            // 这里的取法是"第一次握手完成之前回 0x3703，完成之后回 0"：因为链做完握手会把
            // 状态字 `[0x4000_C81C]` 置 2（ROM `0x0870_3E88`），而睡眠检查（SRAM `0x4000_81FC`）
            // 只在状态==2 时才要求 `& 0xF7F == 0` —— 两个读者的先后顺序就是这把锁的钥匙。
            // **这个 `read` 现在只剩信箱页之后的那半截块会走到**（`0x8001_1000` 往上）：
            // `0x8001_0000..0x8001_0FFF` 整页已改成真内存，值由 `Vm::l1_page_write` 推。
            // 所以"`[l1] 有人读命令字` 一条都没有"**不再等于"没人读它"** —— 固件每拍都读，
            // 只是读的是普通 RAM，不再经过模型。要看谁在读，用 `MT6252_WATCH=0x80010118:4`。
            l1_win::COMMAND => {
                self.trace_reader(pc);
                Some(self.command())
            }
            l1_win::STATUS => Some(self.status),
            l1_win::RESPONSE => Some(self.response),
            _ => None,
        }
    }

    /// 记下是谁在读命令字。`readers` 里最多攒 `READ_TRACES` 个不同 PC，每个新 PC 报一次。
    fn trace_reader(&mut self, pc: u32) {
        let pc = pc & !1;
        if let Some(slot) = self.readers.iter_mut().find(|(at, _)| *at == pc) {
            slot.1 += 1;
            return;
        }
        if self.readers.len() >= READ_TRACES {
            return;
        }
        self.readers.push((pc, 1));
        println!("[l1] 有人读命令字 {:#010x} ← 调用者 PC={:#010x}", l1_win::COMMAND, pc);
    }

    /// AP 对信箱的写。第一笔握手打日志，之后只计数。
    pub fn write(&mut self, addr: u32, value: u32) {
        let first = self.transactions == 0;
        match addr {
            l1_win::REQUEST => {
                let accept = value & l1_win::REQUEST_GO != 0;
                self.status = if accept {
                    self.status | l1_win::STATUS_ACCEPT
                } else {
                    self.status & !l1_win::STATUS_ACCEPT
                };
                if !accept {
                    // 事务结束，应答位跟着撤掉，下一次命令重新置
                    self.response = 0;
                    self.transactions += 1;
                }
                if first {
                    println!("[l1] {}请求，状态字 = {:#010x}", if accept { "收下" } else { "撤掉" }, self.status);
                }
            }
            l1_win::COMMAND if true => {
                // L1 是**按命令作答**的：三条握手分支等的位各不相同（见 `answer_of`），
                // 所以不存在一个能同时满足它们的常量应答。
                let answer = answer_of(value);
                let changed = self.response != answer;
                self.response = answer;
                if self.transactions == 0 && changed {
                    println!("[l1] 收到命令 {value:#06x} → 应答字 = {:#06x}", answer);
                }
            }
            _ => {}
        }
    }
}

/// L1 对某条命令字的作答字。
///
/// 依据是握手例程自己的轮询（ROM `0x0870_49C0` 内，`lsls #n ; bpl` 等的是 bit`31-n` 起，
/// `lsrs #0xf ; bne` 等的是 bit15 落）：
/// - `0x3181` 是第一半条命令，先只举 READY（bit15）；
/// - 第二条写 `0x3080`（`r0==0` 分支，ROM `0x4a32/3a`）：等 bit15 **起** 且 bit12 **起**；
/// - 第二条写 `0x3383`（`r0==3` 分支，ROM `0x4a18/20`）：等 bit15 **起** 且 bit13 **起**；
/// - 第二条写 `0x3783`（`r0==7` 分支，ROM `0x4a00/08`）：等 bit15 **落** 且 bit14 **起**。
///   实测开机走的就是这条，而且它卡住就是因为旧模型给了 0x9000（bit15 还举着）。
fn answer_of(command: u32) -> u32 {
    match command & 0xFFFF {
        0x3080 => l1_win::RESPONSE_READY | l1_win::RESPONSE_DONE,
        0x3383 => l1_win::RESPONSE_READY | l1_win::RESPONSE_DONE_13,
        0x3783 => l1_win::RESPONSE_DONE_14,
        0x3181 => l1_win::RESPONSE_READY,
        _ => l1_win::RESPONSE_READY | l1_win::RESPONSE_DONE,
    }
}
