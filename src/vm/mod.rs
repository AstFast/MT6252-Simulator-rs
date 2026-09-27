pub mod interrupt;
pub mod mmio;
pub mod patch;

use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use unicorn_engine::{
    unicorn_const::{Arch, HookType, Mode, Prot},
    Unicorn,
};

use crate::config::*;
use crate::devices::c1xx::C1xx;
use crate::devices::dma::Dma;
use crate::devices::irqc::IrqCtrl;
use crate::devices::keypad::Keypad;
use crate::devices::l1_win::L1Mailbox;
use crate::devices::lcd::Lcd;
use crate::devices::rtc::Rtc;
use crate::devices::sd::SdCard;
use crate::devices::sfi::SfiFlash;
use crate::devices::sim::SimCard;
use crate::engine::Engine;
use crate::events::{EventReceiver, EventSender, VmEvent, irq_line};
use crate::frame::Shared;
use crate::memmap::{self, Kind, Region, regions};
use crate::profile::Profile;

pub use interrupt::Ctx;

/// 环形缓冲容量
const TRACE_RING: usize = 4096;

/// 运行计数，用于心跳日志；出问题时第一个要看的就是"PC 到底动没动"
#[derive(Debug, Default)]
pub struct Stats {
    pub slices: u64,
    pub irq_ok: u64,
    /// 因为画像/固件没放开这条中断线而被拒
    pub irq_blocked_by_mask: u64,
    /// 因为 CPU 的 CPSR I 位而被拒
    pub irq_blocked_by_cpu: u64,
    /// 因为中断嵌套到顶而被丢弃
    pub irq_nested_full: u64,
    /// 因为调度器还没起来（没有任务在跑）而被丢弃
    pub irq_no_task: u64,
    /// 走到哨兵、真的把一个注入的上下文恢复回去了多少次。与 `irq_ok` **必须配对**：
    /// 长期 `irq_ok - irq_returned > 0` 就说明有注入再也没返回，之后任何一次"返回"都会
    /// 恢复到**上一代**的 SP/PC 上（#46 要抓的就是这个）。
    pub irq_returned: u64,
    /// 哨兵说要返回、但 `ctx_stack` 已经空了的次数。这是"恢复动作发生了两次"的直接证据。
    pub irq_orphan_return: u64,
    /// 已经合成给 UI 的帧数
    pub frames: u64,
}

/// 设备有挂起事件时，一个时间片内每次 `emu_start` 跑多少条指令就去重试投递。
/// 取 2048 是实测折中：256 时一次开机里片数从 ~4400 掉到 ~2780（每片多 59 次 `emu_start`），
/// 而投递延迟只需要"别整片都错过"，2048 已经比原来的 15000 细一个数量级。
const DEV_RETRY_CHUNK: usize = 2048;

/// 单个 VM 的全部可变状态。它被塞进 `Unicorn::new_with_data`，
/// 因此 hook 回调里能直接拿到 `&mut Vm`，不需要 C 版那种全局变量。
pub struct Vm {
    pub lcd: Lcd,
    pub keypad: Keypad,
    pub rtc: Rtc,
    pub irq: IrqCtrl,
    pub dma: Dma,
    pub c1xx: C1xx,
    pub sd: SdCard,
    pub sfi: SfiFlash,
    /// L1 协处理器的信箱窗口（`0x8001_xxxx`），L1D 驱动逐拍校验它的状态字
    pub l1: L1Mailbox,
    pub sim: [SimCard; 2],
    /// 未实现寄存器的后备存储：按 4 字节字稀疏保存，保留固件写入的旧值
    pub reg_store: HashMap<u32, u32>,
    /// 中断嵌套用；C 版是 `isrStackList[10][17]` 的定长数组
    pub ctx_stack: Vec<Ctx>,
    pub callback_ctx: Option<Ctx>,
    pub stats: Stats,
    /// MMIO 读次数计数，用来回答"那个死循环在轮询哪个寄存器"
    pub poll_counts: HashMap<u32, u64>,
    /// 画像里 `kind = "count"` 探针的累加值，用来回答"这条链到底跑了几次"。
    /// 用 BTreeMap 是为了打印顺序稳定、两次运行可比。
    pub counters: BTreeMap<String, u64>,
    /// 这一轮**实际装上**的探针标签（已排序去重）。`MT6252_ZEROS=1` 时用来把没命中的也列成
    /// `标签=0` —— 否则"缺席"和"命中 0 次"在报告里长得一样，而这正是最容易把"我没测到"
    /// 说成"它没发生"的地方。
    pub probe_labels: Vec<String>,
    /// L1 信箱窗口各地址上次打过的写值，用来把每拍都踢的门铃压成状态跳变
    l1_written: HashMap<u32, u32>,
    /// MT6252_TRACE=1 时启用的执行块环形缓冲，用于回答"是怎么走到死循环的"
    trace: Vec<u32>,
    trace_pos: usize,
    pub trace_on: bool,
    /// 实验开关：MT6252_NO_IRQ=1 时完全不注入中断，用来区分"卡死是 ISR 返回坏了还是别的原因"
    pub suppress_irq: bool,
    /// 是否按硬件语义切到 IRQ 模式（关掉就退回 C 版的偷懒做法）
    pub mode_switch: bool,
    /// 未实现设备的读回值覆盖表（来自画像）
    pub read_override: HashMap<u32, u32>,
    /// 当前固件的画像：地址、按键矩阵、屏幕尺寸、补丁全在里面
    pub profile: Profile,
    /// 合成后的帧，UI 线程取用
    pub frame: Shared,
    events: EventReceiver,
    t0: Instant,
    /// 下一次周期 tick / RTC / SIM 注入落在第几个时间片。按片计数而不是比宿主墙钟，
    /// 否则同样的运行两次会走出不同的固件轨迹（见 `config::slices_per` 的说明）。
    next_periodic_tick: Option<u64>,
    next_rtc: u64,
    /// 一个时间片跑多少条 guest 指令（`MT6252_SLICE_INSNS` 可覆盖）
    pub slice_insns: usize,
    /// 虚拟 SIM 注入点已经做过没有（见 `profile::SimStub`），一次开机只来一发
    sim_stub_done: bool,
    /// 13 MHz 计数器，按时间片固定推进（不按宿主墙钟，否则运行不可复现）
    clock13: u32,
    /// `MT6252_KEYS` 解析出来的按键脚本：`(片, 键码, 按下/松开)`，已按片排序。
    /// 走的是和 UI 真按键**同一个** `handle_event` 入口，所以脚本能打通的链路
    /// 就是窗口里能打通的链路，不是旁路。
    key_script: Vec<(u64, u8, bool)>,
    key_pos: usize,
    /// 还没投出去的按键（先进先出）。投不出去的原因只有三类：CPU 关中断、调度器还没起来、
    /// 嵌套到顶；三者都是暂时的，所以每片重试。见 [`Vm::deliver_keys`]。
    pending_keys: std::collections::VecDeque<(u8, bool)>,
    /// L1 信箱页（[`memmap::L1_PAGE_BASE`]）改成真内存之后剩下的写侧状态。
    ///
    /// `REQUEST.go` 上一次的值：命令握手靠的是**边沿**（举起来 = L1 收下，放下 = 事务结束），
    /// 不是电平，所以必须自己记着上一次。
    l1_request_go: bool,
    /// 递归闸。宿主往页里推值时 `uc_mem_write` 会不会又打回本页的写钩子，
    /// **不靠推理断定，用 `l1_api_reentry` 计数实测**：收尾时它是 0 就说明没发生。
    l1_pushing: bool,
    l1_api_reentry: u64,
    /// 页被固件写过多少字、模型往里推过多少次（用来判断改造后回调量真降了多少）
    l1_writes: u64,
    l1_pushes: u64,
    /// `[l1page] 首见写 REQUEST` 那行只打一次（见 `Vm::l1_page_write`）
    l1_order_logged: bool,
}

impl Vm {
    pub fn new(events: EventReceiver, frame: Shared, profile: Profile) -> Self {
        let now = Instant::now();
        let periodic_ms = profile.timer.periodic_ms;
        let sd = SdCard::open(&sdcard_image());
        let read_override = profile
            .read_override
            .iter()
            .map(|o| {
                // 画像里的覆盖地址可以是**半字/字节**地址（C 版就是拿它原样
                // `uc_mem_write(.., 4)` 写进 guest RAM 的，所以 `0xA100_03F6 = 0x8888` 真的落在
                // F6..F7 两字节上，固件 `ldrh [0xA10003F6]` 才读得到 0x8888）。
                // 而 `mmio_read` 读完整字会按访问偏移抽 lane，值必须先摆进对应的 lane，
                // 否则这类覆盖一律读成 0。
                let lane = (o.addr & 3) * 8;
                (o.addr & !3, o.value.wrapping_shl(lane))
            })
            .collect();
        Self {
            lcd: Lcd::new(profile.lcd.width, profile.lcd.height),
            keypad: Keypad { matrix: profile.keypad.matrix.clone(), ..Keypad::default() },
            rtc: Rtc::default(),
            irq: IrqCtrl::default(),
            dma: Dma::default(),
            c1xx: C1xx::default(),
            sim: [SimCard::new(0), SimCard::new(1)],
            reg_store: HashMap::new(),
            ctx_stack: Vec::new(),
            callback_ctx: None,
            stats: Stats::default(),
            poll_counts: HashMap::new(),
            counters: BTreeMap::new(),
            probe_labels: Vec::new(),
            l1_written: HashMap::new(),
            trace: vec![0; TRACE_RING],
            trace_pos: 0,
            trace_on: crate::config::env_on("MT6252_TRACE"),
            suppress_irq: crate::config::env_on("MT6252_NO_IRQ"),
            mode_switch: crate::config::env_on("MT6252_MODESWITCH"),
            sd,
            sfi: SfiFlash::default(),
            l1: L1Mailbox::default(),
            read_override,
            profile,
            events,
            t0: now,
            next_periodic_tick: periodic_ms.map(slices_per),
            next_rtc: RTC_TICK_SLICES,
            clock13: 0,
            key_script: parse_key_script(),
            key_pos: 0,
            pending_keys: std::collections::VecDeque::new(),
            l1_request_go: false,
            l1_pushing: false,
            l1_api_reentry: 0,
            l1_writes: 0,
            l1_pushes: 0,
            l1_order_logged: false,
            sim_stub_done: false,
            slice_insns: std::env::var("MT6252_SLICE_INSNS")
                .ok()
                .and_then(|v| v.parse::<usize>().ok())
                .unwrap_or(EMU_SLICE_INSNS),
            frame,
        }
    }

    /// 建立映射、装入固件、挂好 hook，然后跑起来直到收到退出信号
    pub fn boot(self, rom: Vec<u8>, out: &EventSender) -> Result<(), String> {
        let layout = regions(self.profile.rom.base, self.profile.rom.size);
        let entry = self.profile.entry();
        let vector = self.profile.interrupt.vector;
        let mut emu: Unicorn<'static, Vm> =
            Unicorn::new_with_data(Arch::ARM, Mode::ARM, self).map_err(|e| format!("uc_open: {e}"))?;
        map_memory(&mut emu, &layout)?;
        load_rom(&mut emu, &rom)?;
        arm_stub_page(&mut emu)?;
        patch::install(&mut emu)?;
        install_return_hook(&mut emu)?;
        install_fault_hook(&mut emu)?;
        install_trace_hook(&mut emu)?;
        install_watch_hook(&mut emu)?;
        install_l1_page_hook(&mut emu)?;
        prime_boot(&mut emu);

        println!(
            "Unicorn 就绪，入口 {entry:#x}，IRQ 向量 {vector:#x}，每片 {:#x} 指令，SD {}",
            emu.get_data().slice_insns,
            if emu.get_data().sd.present() { "镜像已挂载" } else { "镜像缺失，文件系统会挂不上" }
        );
        run_loop(&mut emu, out)
    }

    /// 主循环的一个时间片：注入周期性中断、消化 UI 事件、必要时重画屏幕
    fn service(&mut self, eng: Engine) {
        self.stats.slices += 1;
        // 时钟按时间片固定推进：同一个时间片里固件读多少次都是同一个值，所以
        // "连读两次取相等"的安全读天然成立；而每次运行的 guest 时序只取决于时间片数，
        // 不再随宿主负载漂移（按墙钟算时实测同一份代码两次跑，启动深度都不一样）
        self.clock13 = self.clock13.wrapping_add(13 * EMU_SLICE_US as u32);
        let slice = self.stats.slices;
        // CTIRQ1 由固件写的比较值决定何时拉，不按固定周期盲投；
        // 投不进去（CPU 关中断）就保持有效，下个时间片再试，跟硬件一致
        if self.profile.timer.ctirq1 {
            let counter = self.c1xx.counter(self.tick_13mhz());
            if self.c1xx.matched(counter) && self.raise_irq(eng, irq_line::CTIRQ1_13M) {
                self.c1xx.consume(counter);
            }
        }
        // 画像要求的周期 tick：C 版是每 5ms 无条件投一次，投到哪条线由画像决定
        if let Some(due) = self.next_periodic_tick {
            if slice >= due {
                let period = slices_per(self.profile.timer.periodic_ms.unwrap_or(5));
                self.next_periodic_tick = Some(due + period);
                let line = self.profile.timer.line;
                self.raise_irq(eng, line);
            }
        }
        if slice >= self.next_rtc {
            self.next_rtc += RTC_TICK_SLICES;
            self.handle_event(eng, VmEvent::Rtc);
        }
        // 虚拟 SIM 卡：到点补上"没人按下发送键"这一步（见 `profile::SimStub`）
        self.post_sim_request(eng);
        // 卡的挂起事件**每片都要试**。原先按 `SIM_TICK_SLICES`（500 片）才试一次，而
        // `raise_irq` 在 CPSR.I 关着时直接返回 false —— 一片只有"片尾"这一个采样点，
        // 开机期又大量停在关中断的闪存驱动里，于是实测那次 ATR 的 RX 事件在片 3000/3500
        // 连续被挡，固件在 1400 片里再没等到卡的回执。C 版是事件线程每轮都重投，没有这个问题。
        self.pump_sim(eng);
        // 按键脚本：与 UI 真按键走**同一个** `handle_event` 入口，见 `parse_key_script`
        while self.key_pos < self.key_script.len() && self.key_script[self.key_pos].0 <= slice {
            let (_, key, down) = self.key_script[self.key_pos];
            self.key_pos += 1;
            println!("[keys] 片={slice} 注入键码 {key:#x} {}", if down { "按下" } else { "松开" });
            self.handle_event(eng, VmEvent::Keyboard { key, down });
        }
        while let Ok(ev) = self.events.try_recv() {
            self.handle_event(eng, ev);
        }
        // 上一片被挡住没投出去的按键，这一片再试（C 版的"重新入队"就是这个意思）
        self.deliver_keys(eng);
        if self.lcd.dirty {
            let mut guard = self.frame.lock().unwrap_or_else(|p| p.into_inner());
            if self.lcd.composite(eng, &mut guard.pixels) {
                guard.serial += 1;
                self.stats.frames += 1;
                // 每次都报，但只报前 8 帧：`第一帧` 是 LCD **初始化**里那次清帧传送造成的，
                // 图层地址那时候还全是 0；真正有意义的那一帧在后面。只盯第一帧会把
                // "画了但没显示" 误判成 "什么都没画"。
                if self.trace_on && self.stats.frames <= 8 {
                    let n = (self.lcd.width * self.lcd.height) as usize;
                    let px = &guard.pixels[..n.min(guard.pixels.len())];
                    let painted = px.iter().filter(|p| **p != 0xff00_0000).count();
                    println!(
                        "[lcd] 第 {} 帧 {}x{} 非黑像素={}",
                        self.stats.frames, self.lcd.width, self.lcd.height, painted
                    );
                    println!("[lcd] 图层状态 {}", self.lcd.layer_state_line());
                }
                if let Some(dir) = fb_png_dir() {
                    let n = (self.lcd.width * self.lcd.height) as usize;
                    let painted = guard.pixels[..n.min(guard.pixels.len())]
                        .iter()
                        .filter(|p| **p != 0xff00_0000)
                        .count();
                    let path = format!("{dir}/host_f{}_s{}.png", self.stats.frames, self.stats.slices);
                    let _ = std::fs::create_dir_all(dir);
                    match guard.write_png(&path) {
                        Ok(()) => println!("[lcd] 第 {} 帧 {}x{} 非黑像素={painted} → {path}", self.stats.frames, self.lcd.width, self.lcd.height),
                        Err(e) => println!("[lcd] 写 {path} 失败: {e}"),
                    }
                }
            }
        }
    }

    /// 虚拟 SIM 卡的注入点：到点就调固件自己的消息构造器，让它把第一条 SIM 消息发出去。
    /// 为什么需要这一发、以及它"补的是触发而不是跳流程"，见 `profile::SimStub` 的注释。
    fn post_sim_request(&mut self, eng: Engine) {
        let stub = &self.profile.sim_stub;
        if !stub.enable || self.sim_stub_done || self.stats.slices < stub.after_slices {
            return;
        }
        // **任何时候都不许在 ISR 里 hijack**：`start_callback` 会把 LR 换成回调哨兵，
        // 那样中断自己的返回哨兵（`STUB_IRQ_LAND`）就丢了，被打断的任务再也回不来
        // ——实测过这个错：把 require_quiet 整个关掉之后，冷复位是通了，但 `注册点` 从 4 掉回 0。
        // `require_quiet` 只额外决定"要不要连挂起的回调一起让路"：本固件到点已经 park，
        // MSDC 完成回调占住的那份上下文永远不会再被恢复，所以画像里允许覆盖它。
        let hijack_isr = !self.ctx_stack.is_empty();
        let hijack_callback = stub.require_quiet && self.callback_ctx.is_some();
        if hijack_isr || hijack_callback {
            if self.stats.slices % 1000 == 0 {
                println!(
                    "[sim_stub] 片={} 暂不注入：ISR 栈深度={} 挂起回调={}",
                    self.stats.slices,
                    self.ctx_stack.len(),
                    self.callback_ctx.is_some()
                );
            }
            return;
        }
        let (func, args, poke_addr, poke_value) =
            (stub.func, [stub.arg0, stub.arg1, stub.arg2, stub.arg3], stub.msg_id_slot, stub.msg_id);
        self.sim_stub_done = true;
        // 走构造器那条路时才需要改这个全局（它的 msg_id 取自 `[0xF021E900]`）
        if poke_addr != 0 {
            eng.set_u16(poke_addr, poke_value as u16);
        }
        println!(
            "[sim_stub] 片={} 注入：调 {func:#010x}({:#x},{:#x},{:#x},{:#x}){}",
            self.stats.slices,
            args[0],
            args[1],
            args[2],
            args[3],
            if poke_addr == 0 { String::new() } else { format!("，先把 [{poke_addr:#010x}] 摆成 {poke_value:#x}") }
        );
        self.start_callback(eng, func | 1, args);
    }

    /// 打印 `count` 探针的累计结果。必须在心跳里打：带界面的跑法永不退出、`MAX_SLICES`
    /// 之外的中途观察也只能看心跳，只在收尾打的话前一种情况什么都看不到。
    fn print_counters(&self) {
        // 信箱页改造的自检。`重入` 为 0 就说明宿主的 `uc_mem_write` 不会打回本页的写钩子，
        // 那道闸只是保险；`写` 和 `推` 的比值直接反映固件写里有多少需要模型回话。
        println!(
            "[l1page] 信箱页被写={} 模型推值={} 推值时被钩子重入={}",
            self.l1_writes, self.l1_pushes, self.l1_api_reentry
        );
        // 一个计数都没有时，默认就不打这一行；但开了 `MT6252_ZEROS` 必须照打 ——
        // "全部探针一次没命中"正是这个开关唯一要捕捉的场景，早退就等于把读数又变回缺席。
        let zeros = crate::config::env_on("MT6252_ZEROS");
        if self.counters.is_empty() && !zeros {
            return;
        }
        let mut line: Vec<String> = self.counters.iter().map(|(k, v)| format!("{k}={v}")).collect();
        // `MT6252_ZEROS=1`：把"装了但一次没命中"的 count 探针也列成 `标签=0`。
        // 缺席不是读数，`标签=0` 才是 —— 之前判断"某条链一次没进"只能靠前者，很容易把
        // "探针没装上/被筛掉了"读成"固件没走到这里"。默认不开：全探针画像下有 500 多个标签，
        // 每两秒一行的话日志会被零撑爆。
        if zeros {
            line.extend(
                self.probe_labels
                    .iter()
                    .filter(|l| !self.counters.contains_key(l.as_str()))
                    .map(|l| format!("{l}=0")),
            );
            line.sort();
        }
        if line.is_empty() {
            return;
        }
        println!("[count] {}", line.join(" "));
    }

    /// 把攒着的按键按顺序投出去：**先投中断，投进去了才动寄存器**。
    ///
    /// 照的是 C 版 `main.c:961` 的机制：
    /// ```c
    /// case VM_EVENT_KEYBOARD:
    ///     if (StartInterrupt(8, address)) SimulatePressKey(vmEvent->r0, vmEvent->r1);
    ///     else EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);   /* 失败重新入队 */
    /// ```
    /// 顺序和"看返回值"这两点都是要紧的。原来我们是"先写寄存器 → 无条件投一次 → 不看结果"，
    /// 而 `raise_irq` 在 CPU 关中断 / 调度器未起 / 嵌套到顶时返回 false，于是那颗键的**状态已经
    /// 改了但中断永久丢失**。实测：注入三次键，只有最后一次进了 KBD LISR，而读 KP_MEM 的小助手
    /// 从开机（片 280）之后就一次都没再涨 —— 看着像"固件不响应按键"，其实是我们把投递丢了
    /// （与 [[project-mt6252-bypass-decision]] 记的 SIM 投递丢失同一类）。
    ///
    /// `raise_irq` 只布置现场（压上下文、置 `INT_STATUS`、改 PC），guest 要到本片跑完才回来，
    /// 所以"先投后写"不会让 ISR 读到旧值。
    fn deliver_keys(&mut self, eng: Engine) {
        while let Some(&(key, down)) = self.pending_keys.front() {
            if !self.raise_irq(eng, irq_line::KEYPAD) {
                return;
            }
            self.pending_keys.pop_front();
            let changed = self.keypad.press_key(key, down);
            println!(
                "[keypad] 片={} 键码 {key:#x} {}：中断已投，寄存器{}",
                self.stats.slices,
                if down { "按下" } else { "松开" },
                if changed { "有变化" } else { "无变化(重复态)" }
            );
        }
    }

    fn handle_event(&mut self, eng: Engine, ev: VmEvent) {

        match ev {
            VmEvent::Keyboard { key, down } => {
                self.pending_keys.push_back((key, down));
                self.deliver_keys(eng);
            }
            VmEvent::Rtc => {
                // 名义经过时间 = 片数 × 每片毫秒，和 `clock13` 同一把尺子
                let elapsed_ms = self.stats.slices * (EMU_SLICE_US as u64 / 1000);
                self.rtc.tick(elapsed_ms);
                // C 版的 `Update_RTC_Time()` 是用 `uc_mem_write` 把宿主时间**推进内存**
                // （`main.c:283-295`），不是在寄存器读接口现造返回值。区别很实际：
                // 固件自己往这几个口写的值必须能回读，否则它的时间校验
                // （`0x0803_5BF8` 拿读回值与传入的 7 字节逐字节比）永远失败，
                // 就会掉进"重植魔数 + 空转 500 次"的重试环，最多 0x989680 轮 —— 现象是
                // 深度计数冻住、PC 在 `0x0803_5CB9` 空转，看着像"另一个寄存器没建模"。
                for (reg, value) in self.rtc.time_regs() {
                    self.reg_store.insert(reg, value);
                }
                self.raise_irq(eng, irq_line::RTC_SEC);
            }
        }
    }

    /// SIM 卡的周期性收尾：先处理已就绪的 DMA 搬运，再把卡侧挂起的中断送出去。
    /// C 版把这段逻辑塞在渲染线程的 `else if` 分支里，这里收敛成一个显式的 tick。
    /// 有没有卡在等一次还没投出去的中断。跑主循环用它决定要不要把时间片切小。
    fn sim_irq_pending(&self) -> bool {
        self.sim.iter().any(|c| c.irq_pending)
    }

    fn pump_sim(&mut self, eng: Engine) {
        for index in 0..2 {
            let ch = if index == 0 { crate::devices::dma::ChannelId::Sim1 } else { crate::devices::dma::ChannelId::Sim2 };
            if self.dma.channel(ch).configured {
                self.finish_sim_dma(eng, index as u8);
            }
            let due = {
                let card = &self.sim[index];
                card.irq_pending && (card.irq_enable & card.irq_channel) != 0
            };
            if due {
                // 投递前把这次的通道位置进 ISR 状态：固件的 SIM LISR 第一件事就是读
                // `0x8109_0014` 判断"是谁的中断"，读到 0 会立刻当伪中断返回。
                // C 版靠 `uc_mem_write(MTK, SIM1_IRQ_STATUS, &changeTmp, 4)` 把这一位 push 进
                // 当 RAM 用的寄存器（main.c:973），投递成功才算数。
                let (status, line) = (self.sim[index].irq_status | self.sim[index].irq_channel, Vm::SIM_IRQ_LINES[index]);
                self.sim[index].irq_status = status;
                // **只有投出去了才清挂起位**：C 版是 `if (!StartInterrupt(5)) EnqueueVMEvent(…)`
                // 明确重排。原先先清后投，撞上 CPSR.I 或线屏蔽就把这次卡事件永久丢掉
                // （实测：SIM 使能了 [RX][NOATR][RXERR] 之后，注入线统计里只有 CTIRQ1 和 RTC，
                //  卡的 RX 事件被清掉后再也不会重来，ATR 永远回不去）
                let before = self.irq_block_snapshot();
                let ok = self.raise_irq(eng, line);
                // 只清"投出去了"的那一次。写成 `irq_pending = ok` 等于投递失败时把事件也清了，
                // 正是上面这段注释警告的 bug 19（实测：片 2780 被 CPSR.I 挡掉一次之后，
                // 后面 1300 片再也没有第二次尝试）。
                if ok {
                    self.sim[index].irq_pending = false;
                }
                if self.trace_on {
                    println!(
                        "[sim{index}] 投线{} → {} 片={}",
                        line,
                        self.irq_block_reason(before, ok),
                        self.stats.slices
                    );
                }
            }
        }
    }

    /// `raise_irq` 的四条早退路各自的计数快照。用它来把"这次为什么没投进去"归到具体一条，
    /// 而不是对着恒定的心跳计数器猜 —— 心跳里那四个数是全局累计的，一次投递的增量淹在噪声里。
    fn irq_block_snapshot(&self) -> (u64, u64, u64, u64) {
        (
            self.stats.irq_blocked_by_mask,
            self.stats.irq_blocked_by_cpu,
            self.stats.irq_no_task,
            self.stats.irq_nested_full,
        )
    }

    fn irq_block_reason(&self, before: (u64, u64, u64, u64), ok: bool) -> &'static str {
        let now = self.irq_block_snapshot();
        if ok {
            "已投递"
        } else if now.0 > before.0 {
            "被线屏蔽挡住"
        } else if now.1 > before.1 {
            "被 CPSR.I 关中断挡住"
        } else if now.2 > before.2 {
            "调度器还没起来"
        } else if now.3 > before.3 {
            "中断嵌套到顶"
        } else {
            "未知（suppress_irq？）"
        }
    }
}

fn map_memory(emu: &mut Unicorn<'static, Vm>, layout: &[Region]) -> Result<(), String> {
    for r in layout {
        println!("[map] {:08x}+{:06x} {:<5} {}", r.base, r.size, if r.kind == Kind::Ram { "RAM" } else { "MMIO" }, r.desc);
        match r.kind {
            Kind::Ram => emu
                .mem_map(r.base as u64, r.size as u64, Prot::ALL)
                .map_err(|e| format!("mem_map({:08x}): {e}", r.base))?,
            Kind::Mmio => {
                let base = r.base;
                let read = move |emu: &mut Unicorn<'_, Vm>, off: u64, size: usize| -> u64 {
                    let eng = unsafe { Engine::from_raw(emu.get_handle()) };
                    emu.get_data_mut().mmio_read(eng, base + off as u32, size) as u64
                };
                let write = move |emu: &mut Unicorn<'_, Vm>, off: u64, size: usize, value: u64| {
                    let eng = unsafe { Engine::from_raw(emu.get_handle()) };
                    emu.get_data_mut().mmio_write(eng, base + off as u32, size, value as u32);
                };
                emu.mmio_map(r.base as u64, r.size as u64, Some(read), Some(write))
                    .map_err(|e| format!("mmio_map({:08x}): {e}", r.base))?;
            }
        }
    }
    Ok(())
}

fn load_rom(emu: &mut Unicorn<'static, Vm>, bytes: &[u8]) -> Result<(), String> {
    let vm = emu.get_data();
    let (base, cap) = (vm.profile.rom.base, vm.profile.rom.size);
    let len = bytes.len().min(cap as usize);
    if bytes.len() > cap as usize {
        println!("[rom] 镜像 {:#x} 大于画像声明的 {base:#x}+{cap:#x}，按容量截断", bytes.len());
    }
    emu.mem_write(base as u64, &bytes[..len]).map_err(|e| format!("写固件失败: {e}"))?;
    println!("[rom] {len} 字节 → {base:#010x}");
    Ok(())
}

/// 哨兵页填 `b .`（0xEAFFFFFE）。万一某个哨兵地址被真的执行到，
/// CPU 会原地打转而不是跑飞到未映射区，方便定位。
fn arm_stub_page(emu: &mut Unicorn<'static, Vm>) -> Result<(), String> {
    const B_SELF: [u8; 4] = [0xFE, 0xFF, 0xFF, 0xEA];
    let mut page = Vec::with_capacity(STUB_SIZE as usize);
    for _ in 0..STUB_SIZE / 4 {
        page.extend_from_slice(&B_SELF);
    }
    emu.mem_write(STUB_BASE as u64, &page).map_err(|e| format!("填哨兵页失败: {e}"))
}

fn pad(note: &str) -> String {
    if note.is_empty() {
        String::new()
    } else {
        format!("  — {note}")
    }
}

/// 开机前按画像直写一批寄存器/内存（C 版 `RunArmProgram` 里那些"过检测"），
/// 然后长按开机键。地址落在 MMIO 区间时走后备存储，落在普通内存时直写 guest。
fn prime_boot(emu: &mut Unicorn<'static, Vm>) {
    let eng = unsafe { Engine::from_raw(emu.get_handle()) };
    let vm = emu.get_data_mut();
    if crate::config::env_on("MT6252_LIST_PATCHES") {
        for p in &vm.profile.boot.poke {
            println!("  boot  {:#010x} = {:#x} ({} 字节){}", p.addr, p.value, p.size, pad(p.note.as_str()));
        }
        for o in &vm.profile.read_override {
            println!("  读回  {:#010x} = {:#x}{}", o.addr, o.value, pad(o.note.as_str()));
        }
    }
    let pokes: Vec<(u32, u32, u8)> =
        vm.profile.boot.poke.iter().map(|p| (p.addr, p.value, p.size)).collect();
    for (addr, value, size) in pokes {
        vm.prime_write(eng, addr, value, size);
    }
    let power_key = vm.profile.boot.power_key;
    // 开机键是**直接写寄存器**、不投中断的：这时调度器还没起来，`deliver_keys` 一定失败，
    // 键就会永远卡在队列里。固件开机阶段是**轮询** KP 寄存器的（实测片 280 就读到了），
    // 所以这里绕过中断路径。
    vm.keypad.press_key(power_key, true);
    // 信箱页改成真内存之后，"读回调给的常量"必须换成开机时一次预置，否则那几道闸静默失败
    vm.prime_l1_page(eng);
}

/// 给 L1 信箱页挂**写**钩子。读一个钩子都不挂 —— 页是真内存，固件的读就是一条普通 `ldr`，
/// 这正是把自旋圈从宿主回调开销里解放出来的地方（见 [`memmap::L1_PAGE_BASE`]）。
fn install_l1_page_hook(emu: &mut Unicorn<'static, Vm>) -> Result<(), String> {
    let (begin, end) = (
        memmap::L1_PAGE_BASE as u64,
        memmap::L1_PAGE_BASE as u64 + memmap::L1_PAGE_SIZE as u64 - 1,
    );
    emu.add_mem_hook(HookType::MEM_WRITE, begin, end, |emu, _t, addr, size, value| {
        let eng = unsafe { Engine::from_raw(emu.get_handle()) };
        emu.get_data_mut().l1_page_write(eng, addr as u32, size as usize, value as u32);
        true
    })
    .map(|_| ())
    .map_err(|e| format!("挂 L1 信箱页写钩子失败: {e}"))
}

fn install_return_hook(emu: &mut Unicorn<'static, Vm>) -> Result<(), String> {
    let hook = emu.add_block_hook(STUB_BASE as u64, (STUB_BASE + 0x100) as u64, |emu, addr, _| {
        let eng = unsafe { Engine::from_raw(emu.get_handle()) };
        emu.get_data_mut().on_stub_reached(eng, addr as u32);
    });
    hook.map(|_| ()).map_err(|e| format!("装哨兵 hook 失败: {e}"))
}

/// 打印最近执行过的基本块（连续重复的合并成 `地址 xN`）
fn dump_trace(emu: &mut Unicorn<'static, Vm>) {
    let vm = emu.get_data();
    let mut out = String::new();
    let mut count = 0usize;
    let mut last = None;
    for i in 0..TRACE_RING {
        let idx = (vm.trace_pos + i) % TRACE_RING;
        let addr = vm.trace[idx];
        if addr == 0 {
            continue;
        }
        match last {
            Some(a) if a == addr => count += 1,
            Some(a) => {
                push_block(&mut out, a, count);
                last = Some(addr);
                count = 1;
            }
            None => {
                last = Some(addr);
                count = 1;
            }
        }
    }
    if let Some(a) = last {
        push_block(&mut out, a, count);
    }
    println!("[trace] 最近执行块:{out}");
}

fn push_block(out: &mut String, addr: u32, count: usize) {
    use std::fmt::Write;
    if out.len() > 4000 {
        return;
    }
    let _ = write!(out, " {addr:#x}");
    if count > 1 {
        let _ = write!(out, "x{count}");
    }
}

/// 安装执行块跟踪 hook（只在 MT6252_TRACE=1 时），会显著拖慢仿真。
fn install_trace_hook(emu: &mut Unicorn<'static, Vm>) -> Result<(), String> {
    // 块级跟踪会把每个基本块都回调一次，代价极大，单独用 MT6252_TRACE_BLOCKS 打开
    if std::env::var("MT6252_TRACE_BLOCKS").is_err() {
        return Ok(());
    }
    emu.add_block_hook(0, 0xFFFF_FFFF, |emu, addr, _| {
        let vm = emu.get_data_mut();
        vm.trace[vm.trace_pos] = addr as u32;
        vm.trace_pos = (vm.trace_pos + 1) % TRACE_RING;
    })
    .map(|_| ())
    .map_err(|e| format!("装 trace hook 失败: {e}"))
}

/// 每轮一个唯一的文件名标签。dump 文件名原本只有 `{地址}_{长度}_s{片}.bin`，
/// **不带轮次**，于是后一轮的 `s8000` 会静默覆盖前一轮的同名文件 —— 我就这样把上一轮
/// "显存归零"的快照读成了这一轮的，得出一个不存在的结论（实际那一格现在值是满屏）。
/// 优先用 `MT6252_RUN_TAG`（让我能给每轮起有意义的名字），否则退到进程启动时刻。
fn run_tag() -> &'static str {
    static TAG: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    TAG.get_or_init(|| match std::env::var("MT6252_RUN_TAG") {
        Ok(v) if !v.trim().is_empty() => v.trim().to_string(),
        _ => std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| format!("{}", d.as_secs()))
            .unwrap_or_else(|_| "unknown".to_string()),
    })
    .as_str()
}

/// 一次按键默认按住多少个时间片再松开。**实测必须是几百片量级，不能按"人手最短按压"估。**
/// 原来取 25 片（= 50 ms 名义固件时间），结果注入的键虽然一路走到 MMI 派发，界面却完全不动；
/// 同一条脚本只把按住时长改成 600 片（`11500:q@12100`，≈1.2 s），主菜单就打开了
/// （`dump/uikey2/host_f150_s12420.png`）。⇒ 50 ms 对这份固件的按键扫描来说不是一次按压。
/// 一片 = 2 ms 名义固件时间，250 片 = 500 ms，落在真人按压的中间值上。
const KEY_HOLD_SLICES: u64 = 250;

/// `MT6252_KEYS=11500:q,11800:2@11900,12100:f` —— 按**片**触发的按键脚本。
///
/// 为什么按片而不是宿主墙钟：这是唯一能让"注入第 N 键"在两次运行里落在固件同一点上的
/// 做法（同 `MT6252_MAX_SLICES` 的理由）。字符到键码的映射与窗口键盘共用
/// [`crate::devices::keypad::key_of`]，所以脚本打通的链路就是窗口里能打通的链路，不是旁路。
fn parse_key_script() -> Vec<(u64, u8, bool)> {
    match std::env::var("MT6252_KEYS") {
        Ok(v) if !v.trim().is_empty() => build_key_script(&v),
        _ => Vec::new(),
    }
}

/// `MT6252_KEYS` 的解析部分，抽成纯函数好单测（排序和"按下排在松开前面"是行为，不是巧合）
fn build_key_script(spec: &str) -> Vec<(u64, u8, bool)> {
    let mut out = Vec::new();
    for item in spec.split(',') {
        let item = item.trim();
        if item.is_empty() {
            continue;
        }
        let Some((at, key)) = item.split_once(':') else {
            println!("[keys] 忽略不认识的条目 {item:?}（应为 片:键，可选 @松开片）");
            continue;
        };
        let (key, up) = match key.split_once('@') {
            Some((k, u)) => (k.trim(), u.trim().parse::<u64>().ok()),
            None => (key.trim(), None),
        };
        let Some(code) = crate::devices::keypad::key_of(key) else {
            println!("[keys] 键 {key:?} 没有映射（可用 0-9 q e w a s d f z c n m）");
            continue;
        };
        let Ok(down) = at.trim().parse::<u64>() else {
            println!("[keys] {item:?} 的触发片不是数字");
            continue;
        };
        let up = up.unwrap_or_else(|| down.saturating_add(KEY_HOLD_SLICES));
        if up <= down {
            println!("[keys] {item:?} 的松开片 {up} 不晚于按下片 {down}，忽略");
            continue;
        }
        out.push((down, code, true));
        out.push((up, code, false));
    }
    out.sort_by_key(|(slice, _, down)| (*slice, !down));
    if !out.is_empty() {
        println!("[keys] MT6252_KEYS={spec:?} → {} 个动作", out.len());
    }
    out
}

/// `MT6252_FB_PNG=dump` —— 每合成出一帧就往这个目录写一张 PNG。
///
/// 为什么需要它：**显存 dump 不能代替渲染结果**。dump 只有一层一层裸像素，要人自己按
/// `LxWINKEY` 叠回去；叠的时候把图层顺序读反，就会得出"叠加层没合成上来"的错误结论
/// （2026-09-26 实测错过一次，真相是主机合成器早就叠对了）。窗口里显示的到底是什么，
/// 只有从 `Frame::pixels` 落盘才算数。
fn fb_png_dir() -> Option<&'static String> {
    static DIR: std::sync::OnceLock<Option<String>> = std::sync::OnceLock::new();
    DIR.get_or_init(|| match std::env::var("MT6252_FB_PNG") {
        Ok(v) if !v.trim().is_empty() => Some(v.trim().to_string()),
        _ => None,
    })
    .as_ref()
}

/// 把 `0xF01D288C-0x70` 这种写法拆成 (地址部分, 带符号偏移)。
/// 前导 `0x` 里不含 `+/-`，所以从它后面开始找分隔符；偏移本身可以再带 `0x`。
fn split_addr_off(s: &str) -> (&str, i64) {
    let b = s.as_bytes();
    let start = if s.starts_with("0x") || s.starts_with("0X") { 2 } else { 0 };
    match b[start..].iter().position(|&c| c == b'+' || c == b'-') {
        Some(i) => {
            let i = start + i;
            let v = u32::from_str_radix(s[i + 1..].trim_start_matches("0x"), 16).unwrap_or(0) as i64;
            (s.split_at(i).0, if b[i] == b'+' { v } else { -v })
        }
        None => (s, 0),
    }
}

/// `MT6252_DUMP=0x081a1500:0x400,0x4000b200:0x100` —— 卡住时把这些区间导出成裸文件。
/// 有些代码（L1 驱动、INIT_SRAM 里的内核例程）是运行时才被搬进内存的，
/// 直接反汇编 flash 镜像只会看到 0xFF，必须先 dump 再看。
///
/// 条目可以加 `*` 前缀表示**先解一层指针**：`*0xF01D288C:0x70` = 读 `u32 [0xF01D288C]`
/// 拿到基址再 dump 那么多字节。驱动的控制块几乎都是"静态槽位放一个运行期分配的指针"，
/// 不解这一层就只能靠猜结构体落在哪。文件名用**解出来的真实基址**，所以和直接按地址 dump
/// 的产物同名可合并；解出 0（还没分配）时跳过并说明，免得留下一个全 0 的文件被当成"结构体是空的"。
/// 解出来之后还可以再加偏移：`*0xF01D288C-0x70:0xF0`。需要它是因为有些结构的**头寸字段在
/// 指针之前**（固件自己就在用 `puVar3[-0x40 + tail*2]` 这种负下标）。
/// `tag=Some(片数)` 时文件名带 `_s<片>`：周期快照必须能按时间排列对比，否则每次覆盖同一批文件
/// 只留最后一张，就分不出"内核状态在退化"和"一直就这样"。卡住/收尾仍用无片数名。
fn dump_watched(emu: &mut Unicorn<'static, Vm>, tag: Option<u64>) {
    let Ok(spec) = std::env::var("MT6252_DUMP") else { return };
    let eng = unsafe { Engine::from_raw(emu.get_handle()) };
    let _ = std::fs::create_dir_all("dump");
    for item in spec.split(',') {
        let item = item.trim();
        let (item, deref) = match item.strip_prefix('*') {
            Some(rest) => (rest.trim(), true),
            None => (item, false),
        };
        let (addr_text, len_text) = item.split_once(':').unwrap_or((item, "1024"));
        let (addr_text, off) = split_addr_off(addr_text.trim());
        let Ok(addr) = u32::from_str_radix(addr_text.trim_start_matches("0x"), 16) else { continue };
        let Ok(len) = u32::from_str_radix(len_text.trim_start_matches("0x"), 16) else { continue };
        let mut base = if deref { eng.u32(addr) } else { addr };
        if deref && base == 0 {
            println!("[dump] *{addr:#010x} 当前是 0（结构体还没分配），跳过");
            continue;
        }
        base = (base as i64 + off) as u32;
        let bytes = eng.read_bytes(base, len as usize);
        let path = match tag {
            Some(slice) => format!("dump/{base:08x}_{len:x}_r{}_s{slice}.bin", run_tag()),
            None => format!("dump/{base:08x}_{len:x}_r{}.bin", run_tag()),
        };
        match std::fs::write(&path, bytes) {
            Ok(()) => {
                let via = if deref { "* " } else { "" };
                println!("[dump] {via}{addr:#010x} +{len:#x} → {path}")
            }
            Err(e) => println!("[dump] 写 {path} 失败: {e}"),
        }
    }
}

/// `MT6252_WATCH=0x4000b210:4,0x4000b234` —— 监视若干 guest 地址被谁写。
/// 固件自旋等 RAM 标志时，这是唯一能直接给出答案的手段（MMIO 那边有 `[poll]`，RAM 没有）。
fn install_watch_hook(emu: &mut Unicorn<'static, Vm>) -> Result<(), String> {
    let Ok(spec) = std::env::var("MT6252_WATCH") else { return Ok(()) };
    for item in spec.split(',') {
        let item = item.trim().trim_start_matches("0x");
        let (addr, len) = match item.split_once(':') {
            Some((a, l)) => (u32::from_str_radix(a, 16).map_err(|e| format!("{item}: {e}"))?, u32::from_str_radix(l, 16).unwrap_or(4)),
            None => (u32::from_str_radix(item, 16).map_err(|e| format!("{item}: {e}"))?, 4),
        };
        let (begin, end) = (addr as u64, addr as u64 + len as u64 - 1);
        emu.add_mem_hook(HookType::MEM_WRITE, begin, end, move |emu, _t, addr, size, value| {
            let eng = unsafe { Engine::from_raw(emu.get_handle()) };
            // 片号必须自带：没有它，"这条写入发生在什么时候"只能靠日志行号去猜，
            // 而顺序读日志很容易把**另一个对象复用同一地址**的写入当成因果（今天就这么差点误判
            // `0xF01760D2`：写它的是 `0x0807152E` 的分配器，而读它的判别器在片 ~3600）。
            let slice = emu.get_data().stats.slices;
            println!(
                "[watch] {addr:#010x} = {value:#x} ({size} 字节) 由 PC={:#010x} 写入 片={slice}",
                eng.pc()
            );
            let _ = end;
            true
        })
        .map_err(|e| format!("监视 {addr:#x} 失败: {e}"))?;
        println!("[watch] 监视 {addr:#010x}..{end:#010x}");
    }
    Ok(())
}

/// 把"访问未映射地址"变成一条带 PC 的日志。C 版这段是注释掉的，出问题时很难定位。
fn install_fault_hook(emu: &mut Unicorn<'static, Vm>) -> Result<(), String> {
    for (kind, label) in [
        (HookType::MEM_READ_UNMAPPED, "读"),
        (HookType::MEM_WRITE_UNMAPPED, "写"),
        (HookType::MEM_FETCH_UNMAPPED, "取指"),
    ] {
        emu.add_mem_hook(kind, 0, u64::MAX, move |emu, _t, addr, size, value| {
            let eng = unsafe { Engine::from_raw(emu.get_handle()) };
            println!("[fault] {label}未映射地址 {addr:#010x} size={size} value={value:#x}，PC={:#x}", eng.pc());
            // 交给上层：仍然让 emu_start 以 UNMAPPED 错误停下
            false
        })
        .map_err(|e| format!("装 {label} fault hook 失败: {e}"))?;
    }
    Ok(())
}

fn run_loop(emu: &mut Unicorn<'static, Vm>, out: &EventSender) -> Result<(), String> {
    let _ = out;
    let stop = STOP.load(Ordering::Relaxed);
    if stop {
        return Ok(());
    }
    // 第一个时间片从镜像入口开始，之后每个时间片从当前 PC 续跑
    let mut begin = emu.get_data().profile.entry() as u64;
    let mut first = true;
    let mut last_heartbeat = Instant::now();
    let slice_insns = emu.get_data().slice_insns;
    let mut recent = [0u64; 8];
    let mut recent_pos = 0usize;
    let mut stuck = 0u32;
    // `MT6252_DUMP_EVERY=500`：有界跑是被外部 timeout 杀掉的，收尾不会执行，所以想看
    // "卡住之后"的内核状态只能靠周期导出。快照文件名带片号，这样一轮里能拿到**时间序列**，
    // 用来分辨"状态在恶化/在等"和"从头到尾就是这个状态"。
    let dump_every: u64 = std::env::var("MT6252_DUMP_EVERY")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(0);
    // `MT6252_MAX_SLICES`：跑满这么多片就自己退出（0/未设 = 不限，靠 UI 关闭或 STOP 信号）
    let max_slices: u64 = std::env::var("MT6252_MAX_SLICES")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(0);
    // `MT6252_FP=500`：每 500 片打一行轨迹指纹（见 `print_fingerprint`）
    let fp_every: u64 = std::env::var("MT6252_FP")
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(0);
    loop {
        if STOP.load(Ordering::Relaxed) {
            let st = &emu.get_data().stats;
            println!(
                "[vm] 收到退出信号，运行 {:?} 后停止（片={} irq ok={} 挡住 位图={}/CPU={}）",
                emu.get_data().t0.elapsed(),
                st.slices,
                st.irq_ok,
                st.irq_blocked_by_mask,
                st.irq_blocked_by_cpu
            );
            return Ok(());
        }
        // until=0 表示"不设结束地址"，靠跑满 EMU_SLICE_INSNS 条指令回到主循环注入事件。
        // 这里不能用 timeout：那是宿主墙钟，每片切在哪条指令上随宿主负载漂移，
        // 同样的运行两次会走出不同的固件轨迹（见 config::EMU_SLICE_INSNS 的实测记录）。
        //
        // **设备有挂起事件时，把这一片切成小段跑，段间重试投递。** `raise_irq` 在 CPSR.I
        // 关着的时候只能返回 false，而一整片只给"片尾"这一个采样点：开机期大量时间停在
        // 关中断的闪存驱动和内核临界区里，于是一次 ATR 回执被挡一下就顺延 15000 条指令。
        // 实测（片 2780 那一次）就是被 CPSR.I 挡掉后 1300 片没等到第二次机会。
        // C 版没有这个问题，因为它的事件是在自己的线程里连续采样重投的。
        // 切段仍然按指令数截止，所以不破坏可复现性。
        let mut left = slice_insns;
        while left > 0 {
            let n = if emu.get_data().sim_irq_pending() { DEV_RETRY_CHUNK.min(left) } else { left };
            if let Err(e) = emu.emu_start(begin, 0, 0, n) {
                println!("[vm] 仿真中断: {e}，PC={:#x}", emu.pc_read().unwrap_or(0));
                dump_regs(emu);
                // 崩溃时**必须**留下现场：`MT6252_DUMP` 原先只在"卡住 60 次"和跑满片数时落盘，
                // 而崩溃那一刻恰恰最需要那几个字节——没有它，"PC 落在 RAM 数据区"这类结论只能
                // 再跑一遍带周期快照去复现。
                let at = emu.get_data().stats.slices;
                dump_watched(emu, Some(at));
                return Err(format!("{e}"));
            }
            left -= n;
            if left == 0 {
                break;
            }
            let eng = unsafe { Engine::from_raw(emu.get_handle()) };
            emu.get_data_mut().pump_sim(eng);
            // **必须在 `pump_sim` 之后**再取续跑地址：投递会把 PC 改成中断向量，
            // 而 `emu_start(begin,…)` 用的是显式入口 —— 先取 begin 就等于把刚设好的向量
            // 又覆盖回被打断的地方，中断"算投出去了"但 ISR 一条都没执行。
            begin = emu_resume_pc(emu);
        }
        let eng = unsafe { Engine::from_raw(emu.get_handle()) };
        emu.get_data_mut().service(eng);
        begin = emu_resume_pc(emu);
        // "在空转"的判据：自旋循环往往有 3~4 个基本块轮流出入，所以看的是
        // "这一片的入口是否还在最近 8 个入口之内"，而不是等于上一个
        let looping = recent.contains(&begin);
        recent[recent_pos] = begin;
        recent_pos = (recent_pos + 1) % recent.len();
        if looping {
            stuck += 1;
        } else {
            stuck = 0;
        }
        if (stuck == 60 || stuck > 60 && stuck % 1000 == 0) && emu.get_data().trace_on {
            dump_trace(emu);
        }
        if stuck == 60 {
            dump_watched(emu, None);
            let eng = unsafe { Engine::from_raw(emu.get_handle()) };
            println!("[stall] PC={begin:#x} 卡住，该处字节 {:02x?}", eng.read_bytes((begin & !1) as u32, 16));
            dump_regs(emu);
            emu.get_data().print_counters();
        }
        let slices = emu.get_data().stats.slices;
        if fp_every != 0 && (slices % fp_every == 0 || max_slices != 0 && slices >= max_slices) {
            print_fingerprint(emu, begin, slices);
        }
        // `MT6252_MAX_SLICES=3000`：**由进程自己按指令片截止**，而不是靠外部 `timeout` 杀。
        // 两个理由：① 片数是确定量，外部墙钟杀会让每轮跑到的深度不一样，结论就没法比；
        // ② 实测 `timeout --signal=KILL` 在 Git Bash 下只杀掉了 `timeout.exe` 自己，
        //   子进程 `mt6252-sim.exe` 活得好好的 —— 连着几轮之后就攒下好几个孤儿在抢核，
        //   而"这轮怎么深度掉了一半"当时被我错记成了别的原因。收尾（快照、计数汇总）也才跑得完。
        if max_slices != 0 && slices >= max_slices {
            let vm = emu.get_data();
            println!(
                "[bound] 跑满 MT6252_MAX_SLICES={max_slices} 片（{:?}），自己收工",
                vm.t0.elapsed()
            );
            vm.print_counters();
            return Ok(());
        }
        if dump_every != 0 && slices > 0 && slices % dump_every == 0 {
            dump_watched(emu, Some(slices));
        }
        if last_heartbeat.elapsed() >= Duration::from_secs(2) {
            let vm = emu.get_data();
            println!(
                "[tick] 片={} irq ok={} 挡住:位图={}/CPU={}/嵌套={}/无任务={} PC={begin:#010x} CPSR={:#x} enabled_l={:#010x}",
                vm.stats.slices,
                vm.stats.irq_ok,
                vm.stats.irq_blocked_by_mask,
                vm.stats.irq_blocked_by_cpu,
                vm.stats.irq_nested_full,
                vm.stats.irq_no_task,
                eng.cpsr(),
                vm.irq.enabled_bits(),
            );
            emu.get_data().print_counters();
            if vm.trace_on {
                let mut hot: Vec<(u32, u64)> = vm.poll_counts.iter().map(|(k, v)| (*k, *v)).collect();
                hot.sort_unstable_by_key(|(_, n)| *n);
                let top: Vec<String> = hot
                    .iter()
                    .rev()
                    .take(6)
                    .map(|(a, n)| format!("{a:#010x}x{n}"))
                    .collect();
                println!("[poll] {}", top.join(" "));
                // 自旋循环看不出在等什么，把 PC 处的指令直接打出来对照反汇编
                let at = (begin & !1) as u32;
                println!("[code] @{at:#010x} {:02x?}", eng.read_bytes(at, 32));
                let vm = emu.get_data_mut();
                vm.poll_counts.clear();
            }
            last_heartbeat = Instant::now();
        }
        if first {
            first = false;
            println!("[vm] 已进入主循环，等待固件跑开机流程");
        }
    }
}

/// 指纹覆盖的 RAM 窗口。刻意**不含任何 MMIO 区间**：`0x8200_0000`（TDMA 帧号）这类
/// 寄存器是"读一次变一次"的，把它算进摘要就等于用观测动作本身改变被观测的轨迹。
const FP_WINDOWS: &[(u32, usize)] = &[
    (0x4000_0000, 0x1_0000),  // SRAM：中断入口、Nucleus TCB、内核栈
    (0xF015_0000, 0x10_0000), // DRAM：mod2q、队列记录、MMI 全局
    (0x2400_0000, 0x4000),    // L1 信箱共享内存
];

/// `MT6252_FP=N`：每 N 片打一行轨迹指纹，用于改动前后的可复现性校验。
///
/// 为什么另起一行而不是复用 `[tick]`：`[tick]` 是**墙钟 2 秒**心跳（见 run_loop 里
/// `last_heartbeat.elapsed()` 那个判断），采样点本身随宿主负载漂移，同一份二进制同一套
/// 参数跑两轮，`[tick]` 的行数和行内容都不一样 —— 拿它当基线会把"宿主忙不忙"读成
/// "固件轨迹变了"。指纹里的量全部只取决于片数。
fn print_fingerprint(emu: &mut Unicorn<'static, Vm>, begin: u64, slices: u64) {
    let eng = unsafe { Engine::from_raw(emu.get_handle()) };
    let cpsr = eng.cpsr();
    let mut digest = 0xcbf2_9ce4_8422_2325u64;
    let mut covered = 0u64;
    let mut missing = String::new();
    for (base, len) in FP_WINDOWS {
        match eng.try_read(*base, *len) {
            Some(bytes) => {
                covered += bytes.len() as u64;
                for b in bytes {
                    digest ^= b as u64;
                    digest = digest.wrapping_mul(0x1000_0000_01b3);
                }
            }
            None => missing.push_str(&format!(" {base:#010x}")),
        }
    }
    let vm = emu.get_data();
    // SP 与"当前任务"一起打，是为了能直接算出 **SP 有没有落在当前任务的 TCB 里** ——
    // #46 那次跑飞的弹栈起点 0xf0193eec 就是 TCB+0x14，光看 pc 永远看不出这件事。
    let sp = eng.sp();
    // 画像里的 `current_task` 存的是"放 TCB 指针的那个格子"的地址，所以要读**两级**
    // （和 `interrupt.rs` 里 `[irq]` 那行一致）。少解一级拿到的是格子地址本身，
    // 拿它当 TCB 去比 SP 就永远比不中。
    let word = |a: u32| -> u32 {
        eng.try_read(a, 4)
            .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .unwrap_or(0)
    };
    let task = vm
        .profile
        .interrupt
        .current_task
        .map(word)
        .filter(|t| *t != 0 && *t != 0xffff_ffff)
        .map(word)
        .filter(|t| *t != 0 && *t != 0xffff_ffff)
        .unwrap_or(0);
    println!(
        "[fp] 片={slices} pc={begin:#010x} cpsr={cpsr:#x} sp={sp:#010x} 任务={task:#010x} irq={irq}/返{ret}/空返{orphan} 挡=位图{mask}/CPU{cpu}/嵌套{nest}/无任务{notask} ram={covered:#x} 摘要={digest:016x} 用时={secs:.1}{missing}",
        irq = vm.stats.irq_ok,
        ret = vm.stats.irq_returned,
        orphan = vm.stats.irq_orphan_return,
        mask = vm.stats.irq_blocked_by_mask,
        cpu = vm.stats.irq_blocked_by_cpu,
        nest = vm.stats.irq_nested_full,
        notask = vm.stats.irq_no_task,
        secs = vm.t0.elapsed().as_secs_f64(),
        missing = if missing.is_empty() { String::new() } else { format!(" 窗口未映射:{missing}") },
    );
    // SP 落进当前任务的 TCB 本体 = 这个任务已经把栈弹过头、正把内核对象当栈帧用。
    // 单独一行而不是塞进 `[fp]`：那行是逐行 diff 的基线，掺进只在异常时变化的字段会污染比对。
    // 0xb4 是 TCB 的实测大小；范围宽窄不影响判据（要抓的是"SP 跑到内核对象里"这种量级）。
    let tcb_end = task.saturating_add(0xb4);
    if task != 0 && (task..tcb_end).contains(&sp) {
        println!("[spalert] 片={slices} SP={sp:#010x} 落在当前任务 TCB {task:#010x}..{tcb_end:#010x} 内");
    }
}

/// `emu_start` 的 timeout 到期后要从"当前 PC"继续；Thumb 态需要把地址低位补成 1
fn emu_resume_pc(emu: &mut Unicorn<'static, Vm>) -> u64 {
    let eng = unsafe { Engine::from_raw(emu.get_handle()) };
    let mut pc = eng.pc();
    if eng.in_thumb() {
        pc |= 1;
    }
    pc as u64
}

fn dump_regs(emu: &mut Unicorn<'static, Vm>) {
    use unicorn_engine::RegisterARM as R;
    let mut line = String::new();
    for reg in [R::R0, R::R1, R::R2, R::R3, R::R4, R::R5, R::R6, R::R7] {
        line.push_str(&format!("{reg:?}={:#010x} ", emu.reg_read(reg).unwrap_or(0)));
    }
    println!("{line}");
    println!(
        "SP={:#x} LR={:#x} PC={:#x} CPSR={:#x}",
        emu.reg_read(R::SP).unwrap_or(0),
        emu.reg_read(R::LR).unwrap_or(0),
        emu.reg_read(R::PC).unwrap_or(0),
        emu.reg_read(R::CPSR).unwrap_or(0),
    );
}

/// UI 线程用它请求停机
pub static STOP: AtomicBool = AtomicBool::new(false);

pub fn request_stop() {
    STOP.store(true, Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::keypad::code;

    /// 一条 `片:键` 展开成"按下 + 松开"两个动作，且**按下排在松开前面**——
    /// 同一片上如果既有上一条的松开又有这一条的按下，顺序反了就会出现"先按后松"覆盖掉按下
    #[test]
    fn one_entry_expands_to_down_then_up() {
        let s = build_key_script("11500:q");
        assert_eq!(s, vec![(11500, code::SOFT_LEFT, true), (11500 + KEY_HOLD_SLICES, code::SOFT_LEFT, false)]);
    }

    /// 数字键走字符面映射，`@松开片` 能覆盖默认按住时长
    #[test]
    fn digits_and_explicit_release_slice_are_honoured() {
        let s = build_key_script("11800:2@11900");
        assert_eq!(s, vec![(11800, 2, true), (11900, 2, false)]);
    }

    /// 脚本可以整段乱序写，落到 `service()` 里是**按片单调消费**的游标，
    /// 所以解析后必须排序；乱序不排就等于后面的动作永远不会被触发
    #[test]
    fn entries_are_sorted_by_slice_regardless_of_input_order() {
        let s = build_key_script("12100:f,11500:q,11800:2");
        assert!(s.windows(2).all(|w| w[0].0 <= w[1].0), "没按片排序：{s:?}");
        assert_eq!(s.len(), 6);
        assert_eq!(s[0], (11500, code::SOFT_LEFT, true));
        assert_eq!(s[4], (12100, code::OK, true));
    }

    /// 写错的条目只丢那一条，不许把整份脚本清空（否则一次笔误会变成"按键没反应"的假阴性）
    #[test]
    fn bad_entries_are_dropped_one_by_one() {
        let s = build_key_script("abc:q, 11500:x, 11700:q@11700, 11900:6");
        assert_eq!(
            s,
            vec![(11900, 6, true), (11900 + KEY_HOLD_SLICES, 6, false)],
            "三条坏的（片号不是数字 / 键名没映射 / 松开不晚于按下）都该被单独丢掉"
        );
    }

    /// 键名字符大小写不敏感（`key_of` 走 `to_ascii_lowercase`），别把"能按"当成"被丢弃"
    #[test]
    fn key_chars_are_case_insensitive() {
        let s = build_key_script("11600:Q");
        assert_eq!(s.len(), 2);
        assert_eq!(s[0], (11600, code::SOFT_LEFT, true));
    }

    /// 偏移写法要能解析对，否则 `*0xF01D288C-0x70` 会把地址本身解成非法而整条静默跳过 ——
    /// 那看起来就像"这个结构体 dump 不出来"
    #[test]
    fn dump_spec_splits_optional_signed_offset() {
        assert_eq!(split_addr_off("0xF01D288C"), ("0xF01D288C", 0));
        assert_eq!(split_addr_off("0xF01D288C-0x70"), ("0xF01D288C", -0x70));
        assert_eq!(split_addr_off("0xF01D288C+8"), ("0xF01D288C", 8));
        assert_eq!(split_addr_off("f01d288c-10"), ("f01d288c", -0x10));
        // 偏移写坏（不是十六进制）时按 0 处理，而不是把整条丢掉
        assert_eq!(split_addr_off("0xF01D288C-zz"), ("0xF01D288C", 0));
    }
}
