//! 固件画像：把与具体固件绑定的数据从代码里搬出来。
//!
//! 芯片本身（内存布局、中断控制器、LCD/KPD/MSDC/SFI 寄存器语义）对所有 MT6252 都一样，
//! 但"哪个地址是 kal_debug_print""开机要直写哪几个寄存器""键盘矩阵长什么样"是每份固件
//! 各自逆向出来的结果。这些放进 `profiles/*.toml`，按 ROM 的 CRC32 自动匹配，
//! 换固件就是加一个画像文件，不用改 Rust 代码。

use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::config::KB;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Reg {
    R0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    R8,
    R9,
    R10,
    R11,
    R12,
    Sp,
    Lr,
    Pc,
    Cpsr,
}

impl Reg {
    /// Unicorn 的 ARM 寄存器编号。R0..R12 在 `uc_arm_reg` 里是连续的。
    pub fn id(self) -> i32 {
        use unicorn_engine::RegisterARM as R;
        let r0 = i32::from(R::R0);
        match self {
            Reg::R0 => r0,
            Reg::R1 => r0 + 1,
            Reg::R2 => r0 + 2,
            Reg::R3 => r0 + 3,
            Reg::R4 => r0 + 4,
            Reg::R5 => r0 + 5,
            Reg::R6 => r0 + 6,
            Reg::R7 => r0 + 7,
            Reg::R8 => r0 + 8,
            Reg::R9 => r0 + 9,
            Reg::R10 => r0 + 10,
            Reg::R11 => r0 + 11,
            Reg::R12 => r0 + 12,
            Reg::Sp => i32::from(R::SP),
            Reg::Lr => i32::from(R::LR),
            Reg::Pc => i32::from(R::PC),
            Reg::Cpsr => i32::from(R::CPSR),
        }
    }
}

fn size4() -> u8 {
    4
}

/// 补丁动作。每条都对应 C 版 `hookCodeCallBack` 里的一个 case，但写成数据。
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Action {
    /// 命中地址时把某个寄存器改成常量
    SetReg { reg: Reg, value: u32 },
    /// 寄存器之间拷贝（过校验类：让 R2 等于 R0）
    CopyReg { reg: Reg, src: Reg },
    /// 往一个地址写常量（MMIO 或普通内存都支持）
    Poke {
        target: u32,
        value: u32,
        #[serde(default = "size4")]
        size: u8,
    },
    /// 把寄存器的当前值写进一个地址（L1 的 scratch 交接就是这个套路）
    PokeFromReg { target: u32, reg: Reg },
    /// 以寄存器为基址、加偏移写一字节（往结构体里打标记）
    PokeAtReg {
        reg: Reg,
        offset: u32,
        value: u32,
        #[serde(default = "size1")]
        size: u8,
    },
    /// 把 Guest 内存里的 C 字符串打到控制台
    LogStr {
        reg: Reg,
        #[serde(default)]
        label: String,
    },
    /// 固定缓冲区地址的字符串（`mr_sprintf` 落地在固定位置）
    LogStrAt {
        target: u32,
        #[serde(default)]
        label: String,
    },
    LogReg {
        reg: Reg,
        #[serde(default)]
        label: String,
    },
    LogPair {
        reg: Reg,
        reg2: Reg,
        #[serde(default)]
        label: String,
    },
    /// 把 `[reg + offset]` 处的一个整数读出来打（`LogReg` 只能看寄存器，看不到消息内容，
    /// 而这里的常见需求正是"这条消息的 id 是多少" —— id 在 `[msg+6]`，寄存器里没有它）
    LogAt {
        reg: Reg,
        offset: u32,
        #[serde(default)]
        label: String,
        /// 读回宽度 1/2/4，缺省 2（消息 id、模块 id 这些都是 u16）
        #[serde(default = "size2")]
        size: u8,
    },
    /// 只累加不打印：用来给"这条链到底跑了几次"计数。
    /// 里程碑探针若挂在每拍都执行的地址上，`log_*` 会把日志撑到十万行，计数才不会。
    Count {
        #[serde(default)]
        label: String,
    },
}

fn size1() -> u8 {
    1
}

fn size2() -> u8 {
    2
}

impl std::fmt::Display for Action {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Action::SetReg { reg, value } => write!(f, "{reg:?} = {value:#x}"),
            Action::CopyReg { reg, src } => write!(f, "{reg:?} = {src:?}"),
            Action::Poke { target, value, size } => write!(f, "[{target:#010x}] = {value:#x} ({size} 字节)"),
            Action::PokeFromReg { target, reg } => write!(f, "[{target:#010x}] = {reg:?}"),
            Action::PokeAtReg { reg, offset, value, size } => {
                write!(f, "[{reg:?} + {offset:#x}] = {value:#x} ({size} 字节)")
            }
            Action::LogStr { reg, label } => write!(f, "print {label}({reg:?} 指向的字符串)"),
            Action::LogStrAt { target, label } => write!(f, "print {label}([{target:#010x}])"),
            Action::LogReg { reg, label } => write!(f, "print {label}({reg:?})"),
            Action::LogPair { reg, reg2, label } => write!(f, "print {label}({reg:?}, {reg2:?})"),
            Action::LogAt { reg, offset, label, size } => {
                write!(f, "print {label}([{reg:?} + {offset:#x}]) ({size} 字节)")
            }
            Action::Count { label } => write!(f, "count {label}"),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct PatchSpec {
    pub addr: u32,
    pub action: Action,
    /// 逆向笔记，只用于自检时打印
    #[serde(default)]
    pub note: String,
}

#[derive(Debug, Deserialize)]
pub struct Rom {
    pub file: String,
    pub base: u32,
    pub size: u32,
    /// 缺省等于 base
    #[serde(default)]
    pub entry: Option<u32>,
    /// 用于自动匹配画像；不填则只能靠 `MT6252_PROFILE` 显式指定
    #[serde(default)]
    pub crc32: Option<u32>,
    /// 镜像复制步长：整份镜像 = 前 `alias_span` 字节原样复制两遍时才有值（由 [`Profile::load`]
    /// 认出来，不写在 TOML 里）。见 [`crate::vm::patch`] 为什么要照它把探针钉到两个地址。
    #[serde(skip)]
    pub alias_span: Option<u32>,
}

#[derive(Debug, Deserialize)]
pub struct Poke {
    pub addr: u32,
    pub value: u32,
    #[serde(default = "size4")]
    pub size: u8,
    #[serde(default)]
    pub note: String,
}

#[derive(Debug, Deserialize)]
pub struct Boot {
    /// 复位后、启动仿真前直写的寄存器/内存
    #[serde(default)]
    pub poke: Vec<Poke>,
    /// 长按开机的键码
    pub power_key: u8,
}

#[derive(Debug, Deserialize)]
pub struct Lcd {
    pub width: u32,
    pub height: u32,
}

#[derive(Debug, Deserialize)]
pub struct Keypad {
    /// 固件 `key_pad_comm_def->keypad[]`，索引即矩阵位编号，0xFE 表示空位
    pub matrix: Vec<u8>,
}

#[derive(Debug, Deserialize)]
pub struct Interrupt {
    /// ARM 异常向量。留空就是 0x18（IRQ），一般不用改
    #[serde(default = "default_vector")]
    pub vector: u32,
    /// 少数固件走不通真向量时，可以退回"直接跳分发入口"的老做法
    #[serde(default)]
    pub handler: Option<u32>,
    /// 内核"当前任务"指针的**存放处**。填了这个才启用"调度器起来没有"的投递门控。
    /// 注意是两级间接：这个地址里放着一个指针，指针指向的那个字才是当前 TCB ——
    /// 与固件分发器自己的取法一致（`0x4000A198: ldr r0,[pc,#-0x744]` → `0x40009A5C`，
    /// 再 `ldr r5,[r0]`）。
    #[serde(default)]
    pub current_task: Option<u32>,
    /// 固件自己建的 **logical → physical 中断号表**（64 个字节，`ldrb` 按逻辑号索引）。
    /// 开机时 `0x08001B00` 把 ROM 里的 `0x0850FA7C`（physical→logical）反转出来放进这里。
    /// 填了它，注入时才会先把逻辑号换成物理号再写 `INT_STATUS` / 查掩码位 ——
    /// 因为 LISR 表、`INT_STATUS[5:0]`、`INT_MASK_*`、`INT_EOI_*` 用的**全是物理号**。
    /// 不填就当两者相同（恒等映射）。
    #[serde(default)]
    pub line_map: Option<u32>,
}

fn default_vector() -> u32 {
    0x18
}

/// CPU 睡眠的判据。
///
/// 真机上"睡着"的表现是 CPU 停在一个**只能被中断打断**的循环里。这份固件的这个循环在
/// SRAM `0x4000_834C..0x4000_8350`（ROM 镜像 `0x0870_3F4C/50`，就是 `bl 0x4000_822c` 配
/// `b .-8`，没有第二个出口），它是睡眠梯 `0x0851_3920 = {0x1c1,0xea,0x75,0x37,0x19,0xb,0x4,
/// 0x1,0,0,0x4000_8339}` 最深一档的处理函数；整份 ROM 里**没有一条 WFI**（按 ARM `e320f003`
/// 扫过，0 命中），所以"睡眠"就是自旋等中断。
///
/// 于是模拟器必须在这种状态下放行中断，否则 RTC/键盘/GPT 永远投不进去：实测 CPU 停在
/// `PC=0x4000a34c`、`CPSR=0x200001f3`（I 与 F 都屏蔽，且 F 是固件自己 `orr #0xc0` 置的，
/// 不是模拟器的漏），`irq ok` 冻在 980、`挡住:CPU` 一路涨到 4307。
///
/// 判据仍然用 `[0x4000_B25C] != 0` 这个内存字，但要知道它是个**从来没生效过的死钩子**：
/// 实测停车时该字为 **0**（`dump/4000b240_40.bin`），邻居 `[0x4000_B260]` 读回来是
/// `0xF016D2F4` 而不是当年记的 `"RISI"` 魔数，这块地址整体都靠不住。改成"PC 落在上面那个
/// 停车区间就算睡着"跑过一轮 A/B，深度计数一点没动（`时钟递增=975`、`irq ok=980` 两边完全
/// 一样）—— 方向错了：tick 冻住是**致命关机之后的结果**（PC 已停在 `0x081B_935A: b .`
/// 且 I/F 全屏蔽），上游是 `0x8001_0118` 那个自相矛盾的读值，见 `memmap::l1_win::COMMAND`。
#[derive(Debug, Deserialize)]
pub struct Sleep {
    /// 睡眠时非 0、醒来后由固件自己清 0 的那个 CPSR 恢复字
    #[serde(default)]
    pub resume_state: Option<u32>,
    /// CPU 停在这里等中断的地址段（含端点）。见 [`Park`]。
    #[serde(default)]
    pub park: Vec<Park>,
}

/// 一段"CPU 停在这里等中断"的地址。
///
/// 为什么是**两段**而不是一整段：实测睡着时 PC 会落在两处 ——
/// L1D 每拍链/最深档停车循环（`0x4000_8264`、`0x4000_8243`、`0x4000_81FC`、
/// 循环体 `0x4000_834c: bl 0x4000_822c` + `0x4000_8350: b .-8`），
/// 以及它调用的开关中断原语（实测终态 `PC=0x4000_a34c`，ROM 镜像 `0x0870_5F48`）。
/// 这两处**不能并成一段**：中间夹着 `0x4000_A290`，那是我们自己的 ISR 分发入口，
/// 圈进来就等于允许在 ISR 中途再放一条中断。
#[derive(Debug, Deserialize)]
pub struct Park {
    pub lo: u32,
    pub hi: u32,
}

/// L1 协处理器与 AP 侧的握手点。这些是**固件自己分配的 RAM 变量**，不是芯片寄存器，
/// 所以只能进画像。
#[derive(Debug, Deserialize)]
pub struct L1 {
    /// L1D 驱动的"请求挂起位图"：CPU 每向 L1 提一个请求就 `orr` 一位（置位代码在
    /// flash `0x081CEECE`），而**整份 ROM 里没有任何地方清它** —— 真机上是 L1 这个
    /// 总线主设备处理完请求后清掉。13 MHz 服务链（运行时 `0x4000822c`）第 6 关要求
    /// 它为 0，否则每一拍都只踢门铃、不干活，开机就停在等 L1 应答的那三个请求上。
    #[serde(default)]
    pub pending_mask: Option<u32>,
}

/// 系统 tick 的兜底注入。
///
/// 硬件上 CTIRQ1 是比较匹配型（见 `devices/c1xx.rs`）：固件写 `0x82000238`，计数器爬到就拉一次。
/// 但有些初始化链上的任务只被"连续的 tick"推得动，忠实模型下反而会更早停住，
/// 所以留一个可选周期作为画像级兜底 —— 它是权宜措施，不是硬件行为。
#[derive(Debug, Deserialize)]
pub struct Timer {
    #[serde(default)]
    pub periodic_ms: Option<u64>,
    /// 完全不投 CTIRQ1（连比较匹配也不投）。
    /// 有些初始化链一旦被 CTIRQ1 打断就会走进更差的路径，先留这个口子做对照。
    #[serde(default = "default_true")]
    pub ctirq1: bool,
    /// 周期 tick 投到哪条中断线。默认 1（CTIRQ1，硬件语义）；
    /// 填 2 即 C 版 `StartInterrupt(2)` 的投法，见 `irq_line::CTIRQ2_L1D`。
    #[serde(default = "default_tick_line")]
    pub line: u32,
}

fn default_tick_line() -> u32 {
    1
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Deserialize)]
pub struct ReadOverride {
    pub addr: u32,
    pub value: u32,
    #[serde(default)]
    pub note: String,
}

/// 虚拟 SIM 卡的"按下发送键"注入点：在设定的时间片，用现成的回调哨兵机制**调固件自己的函数**，
/// 让真实的固件代码把后续流程跑起来（不伪造队列内容、不伪造寄存器、不跳进函数中段）。
///
/// 墙是：SIM/SIM_2 任务都卡在**第一次**阻塞收信（`0x0800_CAE0: bl 0x0823_9D58`，
/// timeout 写死 −1，永远不会自己醒），而消息投递机制本身是好的（一次开机 80 次投递，
/// 只是没有一条目标是 SIM 的 dst 0x12 / SIM_2 的 0x13）——该发消息的那个按 local id
/// 派发的服务模块（`0x0804_92xx` 一族）从头到尾没被调用过。
///
/// 两条试过的路都记在这，免得重复踩：
/// - `func = 0x081E_18A8`（SIM 消息构造器）：机制是通的 —— 实测 `QUEUE_Send` 80→81、
///   出现 `投递dst_id(0x12)`、`SIM第一次阻塞收信` 2→3（任务确实醒了）。但醒后落到
///   `0x0800CB18: bl 0x0801_4668`，那个分发器还要求 `msg[+6]==1`、`msg[+0x151]==1`、
///   `msg[+0x2C5]==0` 加一次长度校验，构造器的入参凑不出这张上下文。
/// - `func = 0x080D_0EC0`（SIM 接口派发器，现在用的这个）：只要 `r1 != 0`、`r3` = 槽号
///   （由 `0x081A_2A24` 按 `0x0851_C584` 的两项配置解析成 0/1），它自己会
///   `[[0xF0161E0C]+slot*4] → entry[8] → entry[0] = 0x0803_56F9`（冷复位 handler）。
///   外层的 `0x0811_D710` 不能用：它要求一个**已分配**的 SIM_DCL 节点
///   （`[node]=0x5A5A5A5A`、`[node+0x14] != -1`、`[node+0x18]=0xA5A5A5A5`），而节点恰恰
///   是由没跑过的那一步创建的。
#[derive(Debug, Deserialize)]
pub struct SimStub {
    /// 关掉就等于没有这一节，用于 A/B 对照
    #[serde(default)]
    pub enable: bool,
    /// 第几个时间片注入（时间片按 guest 指令数截止，所以这个时刻可复现）
    #[serde(default)]
    pub after_slices: u64,
    /// 要调的固件函数（Thumb 代码会自动补 Thumb 位）
    #[serde(default)]
    pub func: u32,
    /// 传给它的四个入参
    #[serde(default)]
    pub arg0: u32,
    #[serde(default)]
    pub arg1: u32,
    #[serde(default)]
    pub arg2: u32,
    #[serde(default)]
    pub arg3: u32,
    /// 非 0 就先把这个 u16 全局改成 `msg_id`（走构造器那条路时需要：它的 msg_id 取自
    /// `[0xF021E900]`，而 SIM main 收信后拿 `[msg+0]` 与 0x6A 比）
    #[serde(default)]
    pub msg_id_slot: u32,
    #[serde(default)]
    pub msg_id: u32,
    /// true = 除了"不在 ISR 里"之外，还要求没有挂起的回调。
    ///
    /// 不在 ISR 里这一条**永远生效**，不受这个开关控制：`start_callback` 会换掉 LR，
    /// 在 ISR 里插头就把中断自己的返回哨兵丢了，被打断的任务再也回不来 —— 实测踩过：
    /// 曾经把这道守卫整个关掉，冷复位/`L1usim_Reset`/ATR 确实通了，但 `注册点` 从 4 掉回 0。
    ///
    /// 而这个开关管的是"要不要覆盖那份已经永远不会恢复的挂起回调"：本固件到点已经 park 在
    /// 关机 `b .` 上，MSDC 完成回调占着的 `callback_ctx` 不会再被走到，等它就等于永不注入。
    #[serde(default = "default_true")]
    pub require_quiet: bool,
}

fn default_sim_stub() -> SimStub {
    SimStub {
        enable: false,
        after_slices: 0,
        func: 0,
        arg0: 0,
        arg1: 0,
        arg2: 0,
        arg3: 0,
        msg_id_slot: 0,
        msg_id: 0,
        require_quiet: true,
    }
}

#[derive(Debug, Deserialize)]
pub struct Profile {
    pub name: String,
    pub rom: Rom,
    pub boot: Boot,
    pub lcd: Lcd,
    pub keypad: Keypad,
    #[serde(default = "default_interrupt")]
    pub interrupt: Interrupt,
    #[serde(default = "default_timer")]
    pub timer: Timer,
    #[serde(default = "default_sleep")]
    pub sleep: Sleep,
    #[serde(default = "default_l1")]
    pub l1: L1,
    #[serde(default = "default_sim_stub")]
    pub sim_stub: SimStub,
    #[serde(default)]
    pub read_override: Vec<ReadOverride>,
    #[serde(default)]
    pub patch: Vec<PatchSpec>,
}

fn default_timer() -> Timer {
    Timer { periodic_ms: None, ctirq1: true, line: default_tick_line() }
}

fn default_sleep() -> Sleep {
    Sleep { resume_state: None, park: Vec::new() }
}

fn default_l1() -> L1 {
    L1 { pending_mask: None }
}

fn default_interrupt() -> Interrupt {
    Interrupt { vector: default_vector(), handler: None, current_task: None, line_map: None }
}

impl Profile {
    pub fn entry(&self) -> u32 {
        self.rom.entry.unwrap_or(self.rom.base)
    }

    /// 画像文件放在 `profiles/` 下，可以在 exe 目录、也可在项目根目录
    pub fn load(rom_bytes: &[u8], rom_name: &str) -> Result<Self, String> {
        let crc = crc32(rom_bytes);
        let span = mirror_span(rom_bytes);
        // 显式指定画像（新固件还没有画像时用它先跑通）
        if let Ok(explicit) = std::env::var("MT6252_PROFILE") {
            let path = PathBuf::from(explicit);
            let text = std::fs::read_to_string(&path).map_err(|e| format!("读 {path:?} 失败: {e}"))?;
            let mut profile: Profile = toml::from_str(&text).map_err(|e| format!("解析 {} 失败: {e}", path.display()))?;
            profile.rom.alias_span = span;
            println!("[profile] 显式使用 {}（ROM CRC32={crc:#010x}）", profile.name);
            return Ok(profile);
        }
        let candidates = profile_files();
        let mut checked = Vec::new();
        for path in &candidates {
            if !path.is_dir() {
                continue;
            }
            let Ok(entries) = std::fs::read_dir(path) else { continue };
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().and_then(|e| e.to_str()) != Some("toml") {
                    continue;
                }
                let text = std::fs::read_to_string(&path).map_err(|e| format!("读 {path:?} 失败: {e}"))?;
                let mut profile: Profile = toml::from_str(&text)
                    .map_err(|e| format!("解析 {} 失败: {e}", path.display()))?;
                let matched = match profile.rom.crc32 {
                    Some(want) => want == crc,
                    // 没写 CRC 的画像按文件名兜底匹配
                    None => profile.rom.file == rom_name,
                };
                if matched {
                    profile.rom.alias_span = span;
                    println!("[profile] 命中 {}（CRC32={crc:#010x}）", profile.name);
                    return Ok(profile);
                }
                checked.push(format!("{} ({})", path.display(), profile.name));
            }
        }
        Err(format!(
            "没有匹配的固件画像。ROM CRC32={crc:#010x}，文件名 {rom_name:?}；已检查：{}\n\
             新建 profiles/{crc:08x}.toml 即可（字段见 profile.rs 顶部注释）",
            if checked.is_empty() { "无".into() } else { checked.join("、") }
        ))
    }
}

fn profile_files() -> Vec<PathBuf> {
    let exe = std::env::current_exe().unwrap_or_else(|_| PathBuf::from("."));
    let exe_dir = exe.parent().map(Path::to_path_buf).unwrap_or_default();
    let mut dirs = vec![exe_dir.join("profiles")];
    for dir in exe_dir.ancestors() {
        dirs.push(dir.join("profiles"));
    }
    dirs.push(PathBuf::from("profiles"));
    dirs
}

/// IEEE CRC32，用来认固件。16 MB 镜像耗时约 10 ms，只在启动时算一次。
/// 认出"整份镜像 = 同一段字节原样复制两遍"。本项目这份 16 MB 镜像就是前 8 MB 复制两次：
/// 于是**每个函数同时存在于 `base+off` 和 `base+off+8MB`**，而低半部的指针表里有几千处直接
/// 指向高半部那一份 ⇒ 补丁只钉一个别名时，"跑的是另一份"会看起来像"从没执行"（0 命中的假阴性）。
/// 认出来之后由 [`crate::vm::patch`] 负责把每个探针同时钉到两个地址。
fn mirror_span(bytes: &[u8]) -> Option<u32> {
    let half = bytes.len() / 2;
    // 太小的镜像不值得当真（半区 < 4 KB 时相等更可能是巧合）
    if half >= 0x1000 && bytes.len() == 2 * half && bytes[..half] == bytes[half..] {
        Some(half as u32)
    } else {
        None
    }
}

pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// 校验画像里明显会把自己坑哭的字段
pub fn validate(profile: &Profile) -> Result<(), String> {
    if profile.keypad.matrix.is_empty() {
        return Err("keypad.matrix 为空".into());
    }
    if profile.keypad.matrix.len() > 8 * 32 {
        return Err("keypad.matrix 超过 256 项，行线只有 8 组 16 位".into());
    }
    let pixels = u64::from(profile.lcd.width) * u64::from(profile.lcd.height);
    if pixels == 0 || pixels > 8 * 1024 * 1024 {
        return Err(format!("lcd 尺寸不合理：{}x{}", profile.lcd.width, profile.lcd.height));
    }
    if profile.rom.size < 64 * KB {
        return Err(format!("rom.size 只有 {:#x}，不像一份固件", profile.rom.size));
    }
    Ok(())
}
