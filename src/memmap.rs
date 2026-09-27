//! MT6252 的物理地址布局。
//!
//! C 版的做法是"所有区间都映射成真内存 + 用 UC_HOOK_MEM_READ/WRITE 拦寄存器"，
//! 读寄存器还得靠"读完后回写"来伪造返回值。这里改成 Unicorn 原生的 MMIO 后端：
//! 寄存器区间用 `uc_mmio_map` 挂读写回调，未实现的偏移由 [`crate::vm::Vm`] 里的
//! 稀疏后备存储保留旧值，语义与 C 版一致但少了几十万次无用 hook。

use crate::config::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// 普通内存（代码、栈、堆）
    Ram,
    /// 外设寄存器：读写走回调
    Mmio,
}

#[derive(Debug, Clone, Copy)]
pub struct Region {
    pub base: u32,
    pub size: u32,
    pub kind: Kind,
    pub desc: &'static str,
}

use Kind::{Mmio, Ram};

/// L1 信箱窗口所在的 4 KB 页（就是 [`block::L1_MAILBOX`] 那个块的前一页）。
///
/// 这块刻意映射成**真内存**而不是 MMIO：L1D 驱动的深度睡眠循环每拍（62 条指令）
/// 要读这页里的 5 个字 —— 门铃 `+0x114`、命令字 `+0x118`、13 MHz 链的 `+0x300/0x304/0x30C`。
/// 挂读回调时，圈速完全由宿主回调的开销决定（实测一轮里这几千字节能占 20 万次回调）。
/// 改成"模型把值推进页里 + 只挂写钩子"之后，固件的读就是一条普通 `ldr`，一次回调都不产生。
pub const L1_PAGE_BASE: u32 = 0x8001_0000;
pub const L1_PAGE_SIZE: u32 = 4 * KB;

/// 芯片固有的布局，与具体固件无关。Flash 窗口的位置和大小随镜像走，见 [`regions`]。
///
/// MMIO 项必须 4 KB 对齐、大小为 4 KB 的整数倍（`uc_mmio_map` 的硬性要求）。
const CHIP_REGIONS: &[Region] = &[
    Region { base: 0x0000_0000, size: 8 * MB, kind: Ram, desc: "SDRAM / 异常向量 / SRAM" },
    Region { base: 0x01FF_F000, size: MB, kind: Ram, desc: "SDRAM 高位窗口（开机自检会读 0x01FFFFEx）" },
    Region { base: 0x4000_0000, size: 8 * MB, kind: Ram, desc: "INIT_SRAM（中断入口、Nucleus TCB）" },
    Region { base: STUB_BASE, size: STUB_SIZE, kind: Ram, desc: "执行流接管哨兵页" },
    Region { base: 0x2000_0000, size: 4 * MB, kind: Ram, desc: "L1 EMEM（L1D 握手例程取回数据读这里）" },
    Region { base: 0x2400_0000, size: 4 * MB, kind: Ram, desc: "L1 信箱共享内存（L1SM handler / 13 MHz 服务链读这里的应答字）" },
    Region { base: 0x7000_0000, size: MB, kind: Ram, desc: "未确认区间" },
    Region { base: 0xF000_0000, size: 8 * MB, kind: Ram, desc: "Flash 代码段（固件本体解引用处）" },
    Region { base: 0x7800_0000, size: MB, kind: Mmio, desc: "UART 收发 FIFO" },
    Region { base: 0x8000_0000, size: 64 * KB, kind: Mmio, desc: "外设总线 A" },
    Region {
        base: L1_PAGE_BASE,
        size: L1_PAGE_SIZE,
        kind: Ram,
        desc: "L1 信箱窗口：真内存，模型推值 + 只挂写钩子（见 `Vm::l1_page_write`）",
    },
    Region { base: L1_PAGE_BASE + L1_PAGE_SIZE, size: 8 * MB - 64 * KB - L1_PAGE_SIZE, kind: Mmio, desc: "外设总线 A 其余部分" },
    Region { base: 0x8100_0000, size: MB, kind: Mmio, desc: "中断控制器 / DMA / UART / KPD / SIM / RTC / SFI / MSDC" },
    Region { base: 0x8200_0000, size: 4 * MB, kind: Mmio, desc: "C1XX 系统定时器 / TDMA" },
    Region { base: 0x8300_0000, size: MB, kind: Mmio, desc: "音频" },
    Region { base: 0x8400_0000, size: MB, kind: Mmio, desc: "外设总线 C" },
    Region { base: 0x8500_0000, size: MB, kind: Mmio, desc: "外设总线 D" },
    Region { base: 0x9000_0000, size: MB, kind: Mmio, desc: "LCD 显示控制器" },
    Region { base: 0xA000_0000, size: MB, kind: Mmio, desc: "L1 音频协处理器窗口" },
    Region { base: 0xA100_0000, size: MB, kind: Mmio, desc: "DSP 配置窗口" },
    Region { base: 0xA200_0000, size: 4 * MB, kind: Mmio, desc: "DSP 数据窗口" },
    Region { base: 0xA300_0000, size: 4 * MB, kind: Mmio, desc: "DSP 数据窗口 2" },
];

/// 完整布局：芯片固有区间 + 由画像决定的 Flash 窗口
pub fn regions(rom_base: u32, rom_size: u32) -> Vec<Region> {
    let mut out = Vec::with_capacity(CHIP_REGIONS.len() + 1);
    out.push(Region { base: rom_base, size: rom_size, kind: Ram, desc: "NOR Flash 窗口（固件镜像）" });
    out.extend_from_slice(CHIP_REGIONS);
    out
}

/// 这个地址是外设寄存器吗？设备与补丁要据此选择"写后备存储"还是"写 guest 内存"
pub fn is_mmio(addr: u32) -> bool {
    CHIP_REGIONS
        .iter()
        .any(|r| r.kind == Mmio && (r.base..r.base + r.size).contains(&addr))
}

/// 外设寄存器块（按 `addr >> 16` 分派）
pub mod block {
    pub const SYSTEM_A: u32 = 0x8000;
    pub const INTC: u32 = 0x8101;
    pub const DMA: u32 = 0x8102;
    pub const UART1: u32 = 0x8103;
    pub const UART2: u32 = 0x8104;
    pub const UART3: u32 = 0x8105;
    pub const GPT: u32 = 0x8106;
    pub const KPD: u32 = 0x8107;
    pub const SIM1: u32 = 0x8109;
    pub const SFI: u32 = 0x810A;
    pub const RTC: u32 = 0x810B;
    pub const MSDC: u32 = 0x810E;
    pub const SIM2: u32 = 0x810F;
    pub const SYS_TIMER: u32 = 0x8200;
    /// L1 协处理器的信箱/EMEM 窗口（`0x8001_0000`）。注意它和 `SYSTEM_A`（`0x8000_xxxx`）
    /// 不是同一个 64K 块，按 `addr >> 16` 分派时不能混用
    pub const L1_MAILBOX: u32 = 0x8001;
    pub const LCD: u32 = 0x9000;
}

/// 中断控制寄存器（0x8101_xxxx）
pub mod intc {
    pub const INT_MASK_SET_L: u32 = 0x8101_0090;
    pub const INT_MASK_SET_H: u32 = 0x8101_0094;
    pub const INT_MASK_CLR_L: u32 = 0x8101_0080;
    pub const INT_MASK_CLR_H: u32 = 0x8101_0084;
    pub const INT_MASK_STA_L: u32 = 0x8101_0070;
    pub const INT_MASK_STA_H: u32 = 0x8101_0074;
    pub const INT_STATUS: u32 = 0x8101_00D8;
    pub const INT_EOI_L: u32 = 0x8101_00A0;
    pub const INT_EOI_H: u32 = 0x8101_00A4;
    /// LISR 派发**真正**用的 ack。L1D 的派发器（`0x0800_2E38`）在 `blx` 完例程之后走
    /// `0x0800_2E90 → 0x0800_3874`，把已服务的下标写进这里 —— 而不是写 `INT_EOI_L`。
    /// 全 ROM 没有任何 `INT_send_EOI(1)`，所以 CTIRQ1 只能靠这个地址清状态。
    pub const INT_ACK: u32 = 0x8101_00DC;
    pub const FIQ_FEOI: u32 = 0x8101_00D4;
}

/// LCD 控制器寄存器（0x9000_xxxx）
///
/// 这里只列设备真正用到的那几个。完整划分见 `devices/lcd.rs` 的模块头：每层一个
/// 0x30 字节的块（层 0 块基址 0x9000_00B0），下面四个地址寄存器都是块内 +0x0C 的
/// `LCD_LxWINADD`，同块还有 WINCON(+0x00) / WINKEY(+0x04) / WINOFS(+0x08) /
/// WINSIZE(+0x10) / WINPITCH(+0x1C)。**别把 0xBC "顺手改成" 0xB0**——0xB0 是控制寄存器。
pub mod lcd_reg {
    pub const FRAME_TRANSFER: u32 = 0x9000_000C;
    /// `LCD_CON`（数据手册页 363）。它的 **bit0 = `LCD_ON`，全 ROM 从不写、只读**，所以只能由
    /// 硬件给。读者两处，**要求相反**：
    /// - `0x0839_AF00`（`sub_839AEC8` 的软件复位握收）：`while ((u16)[0x90000000] & 1) ;` —— 要它**为 0**；
    /// - `0x0813_1E3A`（GDI 提交门）：bit0 **为 1 就直接提交**；为 0 才退而去问显示状态字节
    ///   `[0xF01D2A5C] ∈ {0x17,0x18}`（`bl 0x081E1F3C`），也不是才 return。
    ///
    /// ⇒ 恒答 0 只满足握收，另一侧必须靠状态字节放行（实测状态字节停在 2/6，所以图层从未提交）。
    /// 恒答 1 会把握收 turned 成永久死循环（实测吃掉片 2830 之后的全部深度）。帧传送是另一个
    /// 寄存器 `+0xC`，别混。**这条相反关系我先前写成"两个读者都要 0"，据此回退过一次状态机尝试。**
    pub const DISP_STAT: u32 = 0x9000_0000;
    /// `DISP_STAT` 的 bit0：`LCD_ON`，显示引擎开着。
    pub const DISP_STAT_RUNNING: u32 = 0x1;
    pub const L0_ADDRESS: u32 = 0x9000_00BC;
    pub const L1_ADDRESS: u32 = 0x9000_00EC;
    pub const L2_ADDRESS: u32 = 0x9000_011C;
    pub const L3_ADDRESS: u32 = 0x9000_014C;
}

/// L1 协处理器的信箱窗口寄存器（`0x8001_xxxx`）。
///
/// 分派按 `addr >> 16` 走，所以这里的地址都是完整字地址。哪一位要被置成什么，
/// 全部来自 L1D 驱动里的比较指令（运行时 `0x4000822c` 的逐拍条件链、
/// `0x40008DC4` 的命令握手），不是数据手册 —— PDF 在这台机器上取不出文本。
pub mod l1_win {
    /// AP → L1 的请求字。握手例程先等 [`STATUS`] 的 busy 位清零，再往这里写 bit0
    pub const REQUEST: u32 = 0x8001_0030;
    pub const REQUEST_GO: u32 = 0x0001;
    /// L1 → AP 的状态字，**由 L1 侧写**：bit15 = 忙，bit0 = 已接受请求
    pub const STATUS: u32 = 0x8001_0034;
    pub const STATUS_BUSY: u32 = 0x8000;
    pub const STATUS_ACCEPT: u32 = 0x0001;
    /// 门铃：固件每拍对它做 read-modify-write，**写入值本身就是走没走到干活分支的探针**
    /// （`0x400082b2` 干活路径 `orrs #3`，`0x400082ba` bail 路径 `orrs #1`）
    pub const DOORBELL: u32 = 0x8001_0114;
    // 这个窗口里还有一个 `0x8001_0010`，**bit0 = "L1 已经起来"**。固件在 `0x0800_0FE8` 把
    // bit0 抄进 `[0xF016AC66]`，而 `0x0800_73C8` 判"这个字不是 1 就返回非 0"，那个非 0 会让
    // `0x0800_10C8` 往 PWDKEY 写关机请求 —— 开机中途那条 `PWDKEY写入(0x2205)` 的根源就是它。
    // 实测**不要**替 L1 把这个位置起来：置 1 之后关机请求确实消失，但固件改走"真依赖 L1"
    // 那条分支，30 秒内连 `SD mount ok!` 都印不出来（卡在 RTC 计时的初始化里）。
    // C 版这个寄存器同样读 0，而 C 版能出图，所以这里保持不建模。
    /// AP → L1 的命令字，一次握手连写两个（先 `0x3181`，再 `0x3080`/`0x3383`/`0x3783`），
    /// 写点在 ROM `0x0870_49FA/FC`、`0x4a12/16`、`0x4a2a`（`ldr r4,[=0x80010020]` →
    /// `adds r6,0xE0` → `strh r5,[r6,#0x18]`，所以这个地址永远不会作为字面量出现，扫不到）。
    ///
    /// **这个寄存器上挂着两个互相矛盾的读者，是当前的墙**：
    /// - 空闲/睡眠检查在 SRAM `0x4000_81FC`（ROM `0x0870_3DF8`；SRAM−ROM 的固定差是
    ///   `0x3790_4404`，与 `0x4000_8264↔0x0870_3E60`、`0x4000_8DC4↔0x0870_49C0` 两处独立印证）做
    ///   `ldrh r0,[0x80010118] ; movs r1,0xf7f ; tst r0,r1 ; bne` —— 要这些位**为 0**；
    ///   （此处以前记成 `0x4000_820E`，那个地址按同一个差换算不出 `0x0870_3DF8`，是错的）
    ///   非 0 就上报 trace point `0x172` → 记录器 `0x0823_D72C` → 只增不清的
    ///   `[0xF016AD1C]` → 第二次直接 "Possibly Endless Nested Exceptions" → 关机。
    /// - 另有一个读者在举 `REQUEST.go` **之前**要它读回非 0：把它改成"读最后写入值/
    ///   事务没开着就返回 0"实测两次（常量改锁存、以及按事务开关门控），
    ///   致命路径都消失，但 `[l1] 收下请求` 再也不出现、握手一次都开不起来，
    ///   深度计数一点没涨（`时钟递增=975`、`QUEUE_Send=80`、`起来.X` 44 全同）。
    /// 所以恒返回 [`COMMAND_ANSWER`] 是"两害相权"的当前选择：它换来握手能跑，
    /// 代价是睡眠检查必然判失败。**真机一定是在这两次读之间让这个字变过**
    /// （L1 是这块窗口的总线主设备），下一步就是查是谁、按什么事件改它。
    pub const COMMAND: u32 = 0x8001_0118;
    pub const COMMAND_ANSWER: u32 = 0x0000_3703;
    /// L1 → AP 的应答字（`0x8001_0700` 窗口 +0x14）。命令被取走后 L1 置起 bit15、bit12，
    /// 握手例程两个都等（`lsls #0x10` / `lsls #0x13` 配 `bpl` 原地转）
    pub const RESPONSE: u32 = 0x8001_0714;
    pub const RESPONSE_READY: u32 = 0x8000;
    /// 命令 `0x3080` 的完成位（`r0==0` 分支等它）
    pub const RESPONSE_DONE: u32 = 0x1000;
    /// 命令 `0x3383` 的完成位（`r0==3` 分支等它）
    pub const RESPONSE_DONE_13: u32 = 0x2000;
    /// 命令 `0x3783` 的完成位（`r0==7` 分支等它，而且同一条分支还要求 READY 已经落下）
    pub const RESPONSE_DONE_14: u32 = 0x4000;
    /// 13 MHz 服务链第 2、3 关：同一个基址 `0x8001_0300` 上的三个字，各校验一个 bit，
    /// **位移不一样**：`+0x00` 是 `lsls r1,#0x1c`（bit3 须为 1）、`+0x04` 是 `#0x18`
    /// （bit7 须为 1）、`+0x0C` 是与 `0xF807` 做等值比较。其余位含义未知，先给 0
    pub const READY_A: u32 = 0x8001_0300;
    pub const READY_A_VALUE: u32 = 0x0008;
    pub const READY_B: u32 = 0x8001_0304;
    pub const READY_B_VALUE: u32 = 0x0080;
    pub const STATE: u32 = 0x8001_030C;
    pub const STATE_VALUE: u32 = 0x0000_F807;
}

/// MSDC（SD 卡）寄存器（0x810E_xxxx）
pub mod msdc_reg {
    pub const CMD: u32 = 0x810E_0024;
    pub const ARG: u32 = 0x810E_0028;
    pub const DATA_STAT: u32 = 0x810E_002C;
    pub const CMD_STAT: u32 = 0x810E_0040;
    pub const DATA_RESP0: u32 = 0x810E_0030;
    pub const DATA_RESP1: u32 = 0x810E_0034;
    pub const DATA_RESP2: u32 = 0x810E_0038;
    pub const DATA_RESP3: u32 = 0x810E_003C;
    pub const CMD_RESP0: u32 = 0x810E_0000;
    pub const CMD_RESP1: u32 = 0x810E_0004;
    pub const CMD_RESP2: u32 = 0x810E_0008;
    pub const CMD_RESP3: u32 = 0x810E_000C;
    pub const DAT_STA: u32 = 0x810E_0044;
}

/// DMA 寄存器：通道块基址 + 通道内偏移
pub mod dma_reg {
    pub const BASE: u32 = 0x8102_0000;
    pub const GLBSTA: u32 = 0x8102_0000;
    /// 每个通道占 0x100
    pub const CHANNEL_BLOCK: u32 = 0x100;
    pub const MSDC_CHANNEL: u32 = 0x200;
    pub const SIM1_CHANNEL: u32 = 0x300;
    pub const SIM2_CHANNEL: u32 = 0x400;

    pub const OFF_TRANSFER_COUNT: u32 = 0x10;
    pub const OFF_CONTROL: u32 = 0x14;
    pub const OFF_START: u32 = 0x18;
    pub const OFF_INTSTA: u32 = 0x1C;
    pub const OFF_DATA_ADDR: u32 = 0x2C;
}

/// 键盘矩阵控制器（0x8107_xxxx）
pub mod kpd {
    pub const BASE: u32 = 0x8107_0000;
}

/// RTC（0x810B_xxxx）
pub mod rtc_reg {
    pub const BASE: u32 = 0x810B_0000;
    /// 中断控制/状态：2 = 秒计数器中断，1 = 闹钟中断
    pub const IRQ_CONTROL: u32 = 0x810B_0000;
    pub const IRQ_STATUS: u32 = 0x810B_0004;
    pub const SEC: u32 = 0x810B_0014;
    pub const MIN: u32 = 0x810B_0018;
    pub const HOUR: u32 = 0x810B_001C;
    pub const DAY: u32 = 0x810B_0020;
    pub const WEEK: u32 = 0x810B_0024;
    pub const MONTH: u32 = 0x810B_0028;
    pub const YEAR: u32 = 0x810B_002C;
}

/// SPI Flash 控制器（0x810A_xxxx）
pub mod sfi {
    pub const MAC_CTL: u32 = 0x810A_0000;
    pub const OUTPUT_LEN: u32 = 0x810A_0004;
    pub const INPUT_LEN: u32 = 0x810A_0008;
    pub const GPRAM_DATA: u32 = 0x810A_0800;
    /// Flash 忙标志（真机在 0x8301_0A28）
    pub const BUSY: u32 = 0x8301_0A28;
}

/// C1XX 系统定时器与 TDMA（0x8200_xxxx / 0x8205_0000）
pub mod systimer {
    /// 13MHz 定时器的一对**只写 32 位数据**（hi16 / lo16，唯一写者 `0x08034FAC`，用于
    /// L1SM 睡眠唤醒的时刻补偿）。名字叫 `INT_STATUS` / `INT_MODE` 是沿用 C 版的常量名，
    /// **不是**中断状态：全 ROM 对这两个地址零读取，ISR 判断中断源靠中断控制器的
    /// `INT_STATUS`（`0x810100D8`）。别按"读到即清"建模，详见 `devices/c1xx.rs` 头注释。
    pub const INT_STATUS: u32 = 0x8200_0200;
    pub const INT_MODE: u32 = 0x8200_0204;
    pub const TICK: u32 = 0x8200_0230;
    /// 比较匹配值：计数器爬到这里就拉 CTIRQ1（1 号线）
    pub const COMPARE: u32 = 0x8200_0238;
    pub const CTRL_21C: u32 = 0x8200_021C;
    pub const CTRL_224: u32 = 0x8200_0224;
    pub const CTRL_228: u32 = 0x8200_0228;
    /// 固件写 1 后需要立刻变回 0 的握手位
    pub const HANDSHAKE: u32 = 0x8205_0000;
    /// TDMA 帧号（16 位）。真机由 L1 侧驱动，固件里 `while ((u16)[0x82000000] != 目标)`
    /// 这类"等时隙"的循环全靠它往前走（C 版把它叫 `TMDA_BASE`，并用补丁把目标值直接
    /// 写进去蒙过去）。计数速率是 13 MHz/12，所以一帧 4.615 ms 正好走 5000。
    pub const TDMA_FRAME: u32 = 0x8200_0000;
    /// 帧间隔（FNIT）：固件写它就等于重启 TDMA 时序，帧号从那一刻重新起算
    pub const TDMA_FNIT: u32 = 0x8200_0004;
}

/// 通用定时器（0x8106_xxxx）
pub mod gpt {
    pub const BASE: u32 = 0x8106_0000;
    pub const CON1: u32 = 0x8106_0000;
    pub const VAL1: u32 = 0x8106_000C;
    pub const CTRL_0010: u32 = 0x8106_0010;
}

/// UART（0x8103/0x8104/0x8105 为寄存器，0x7800_0000 段为 DMA 收发 FIFO）
pub mod uart {
    pub const BASE1: u32 = 0x8103_0000;
    pub const LINE_STATUS: u32 = 0x8103_0014;
    pub const FIFO_BASE: u32 = 0x7800_0000;
}

/// 零散的调试用寄存器（按绝对地址匹配）
pub mod misc {
    pub const POWER_DOWN_CON0: u32 = 0x8100_0320;
    /// C 版注释：写 3 表示跳过最开始的地址映射阶段
    pub const ADDR_MAP_CTRL: u32 = 0x8100_0040;
    pub const DEBUG_CTRL: u32 = 0x8001_0008;
    pub const CTRL_810C0090: u32 = 0x810C_0090;
    pub const CTRL_A10001D4: u32 = 0xA100_01D4;
    pub const CTRL_A10003F6: u32 = 0xA100_03F6;
    /// L1 音频协处理器握手位，`sub_8094040` 会一直读它
    pub const L1_HANDSHAKE: u32 = 0xA000_0000;
}

/// SIM 卡控制器寄存器（MT6252 有 SIM1/SIM2 两套，基址不同）
pub mod sim_reg {
    pub const BASE1: u32 = 0x8109_0000;
    pub const BASE2: u32 = 0x810f_0000;
    pub const OFF_CONTROL: u32 = 0x00;
    pub const OFF_IRQ_ENABLE: u32 = 0x10;
    pub const OFF_IRQ_STATUS: u32 = 0x14;
    pub const OFF_TIDE: u32 = 0x24;
    pub const OFF_DATA: u32 = 0x30;
    pub const OFF_COUNT: u32 = 0x34;
    pub const OFF_TOUT: u32 = 0x48;
    pub const OFF_INS: u32 = 0x60;
    pub const OFF_SW1: u32 = 0x68;
    pub const OFF_SW2: u32 = 0x6C;
    pub const OFF_CARD_TYPE: u32 = 0x70;
    pub const OFF_STATUS: u32 = 0x74;
}
