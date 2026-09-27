//! 中断注入与执行流接管。
//!
//! 思路与 C 版一致但不手写寄存器数组：把 LR 指向哨兵页，固件"返回"时由
//! `vm/mod.rs` 里的块 hook 还原始上下文；保存/恢复交给 Unicorn 的
//! `uc_context_*`，顺带把 IRQ 模式的银行寄存器也带上，比 C 版手抄 17 个寄存器更稳。

use unicorn_engine::RegisterARM as R;

use crate::config::{STUB_CALLBACK_RETURN, STUB_CB_LAND, STUB_IRQ_LAND, STUB_IRQ_RETURN};
use crate::devices::irqc::IrqCtrl;
use crate::engine::Engine;
use crate::events::irq_line;
use crate::vm::Vm;

/// 一次中断或回调的 CPU 快照
///
/// `lr` 存的是**被打断那一刻的真 LR**。C 版 `SaveCpuContext` 读的就是活的 R14，
/// 只把"外面那份"的 R14 换成哨兵；快照里如果被写成哨兵地址，ISR 返回后被打断代码
/// 的 R14 就永久变成了 `0x5000_0004`，紧接着的 `BX LR` 会把控制流交回哨兵 hook。
#[derive(Debug, Clone, Copy, Default)]
pub struct Ctx {
    pub cpsr: u32,
    pub r: [u32; 13],
    pub sp: u32,
    pub lr: u32,
    pub pc: u32,
}

/// 允许的中断嵌套层数，超过就丢弃（C 版是固定 10 行数组）
const MAX_NESTING: usize = 8;
const MODE_IRQ: u32 = 0x12;
const IRQ_DISABLE_BIT: u32 = 1 << 7;

/// R0..R12，与固件 ARM 过程调用约定里需要保全的通用寄存器一致
const GPRS: [R; 13] = [
    R::R0, R::R1, R::R2, R::R3, R::R4, R::R5, R::R6, R::R7, R::R8, R::R9, R::R10, R::R11, R::R12,
];

impl Ctx {
    fn capture(eng: Engine) -> Self {
        let mut ctx = Self {
            cpsr: eng.cpsr(),
            r: std::array::from_fn(|i| eng.reg(GPRS[i])),
            sp: eng.sp(),
            lr: eng.lr(),
            pc: eng.pc(),
        };
        if ctx.cpsr & 0x20 != 0 {
            ctx.pc |= 1;
        }
        ctx
    }

    fn restore(self, eng: Engine) {
        // 先回原模式，后面这些 SP/LR 才会写进正确的银行
        eng.set_cpsr(self.cpsr);
        for (reg, value) in GPRS.iter().zip(self.r) {
            eng.set_reg(*reg, value);
        }
        eng.set_reg(R::SP, self.sp);
        eng.set_reg(R::LR, self.lr);
        eng.set_pc(self.pc);
    }
}

impl Vm {
    /// CPU 是否处于"已睡眠、等唤醒"状态。
    ///
    /// 两条判据并列，因为单独任何一条都不成立：
    /// - `[0x4000_B25C] != 0` 是当年按"固件存的唤醒用 CPSR"写的，实测停车时它是 **0**
    ///   （`dump/4000b240_40.bin`），从来没生效过。留着是为了将来换固件时还能用。
    /// - PC 落在画像 `[sleep] park` 给的任一段停车地址里。这条才是真凭据：那段 ROM 镜像里
    ///   **没有一条 WFI**（按 `e320f003` 扫过 0 命中），最深档处理函数 `0x4000_8338` 的体是
    ///   `bl 0x4000_822c` + `b .-8`（除了中断没有出口），而实测采样 PC 并不在循环头上，
    ///   而是落在被调链内部（`0x4000_8264`/`0x4000_8243`/`0x4000_81FC`）和它调用的
    ///   开关中断原语里（终态 `0x4000_a34c`）——所以地址要开成**两段**，见 [`Park`]。
    ///
    /// 要求 `ctx_stack` 为空是必须的：同一条 L1D 链**每拍的 ISR 上下文里也跑**，
    /// 不区分就会在 ISR 中途再放一条中断进来。
    fn asleep(&self, eng: Engine) -> bool {
        if !self.ctx_stack.is_empty() {
            return false;
        }
        if self.profile.sleep.resume_state.is_some_and(|addr| eng.u32(addr) != 0) {
            return true;
        }
        let pc = eng.pc() & !1;
        self.profile.sleep.park.iter().any(|r| r.lo <= pc && pc <= r.hi)
    }

    /// 调度器是否已经起来（也就是"当前有没有一个任务在跑"）。
    ///
    /// RTOS 启动之前 CPU 跑的是 boot 线程，它没有 TCB：这时候投一个中断，内核在 ISR 结尾
    /// 做任务切换，就没有任何东西能把 boot 线程恢复回来 —— 实测投一次 RTC 就把整条启动链
    /// 永久丢掉。真机在这段路上也是保持 IRQ 屏蔽的，所以"没任务就不投"不是权宜而是还原硬件。
    /// 取法与固件分发器一致：`[current_task]` 里放的是指针，指针指向的字才是当前 TCB。
    fn task_running(&self, eng: Engine) -> bool {
        match self.profile.interrupt.current_task {
            None => true,
            Some(slot) => {
                let tcb_ptr = eng.u32(slot);
                tcb_ptr != 0 && eng.u32(tcb_ptr) != 0
            }
        }
    }

    /// 逻辑中断号 → 物理中断号。
    ///
    /// 固件的 LISR 表、`INT_STATUS[5:0]`、`INT_MASK_*`、`INT_EOI_*` 用的全是**物理**号，
    /// 而 `IRQ_Register_LISR` 收的是**逻辑**号，中间隔着一张开机时由 `0x08001B00` 从 ROM
    /// `0x0850FA7C` 反转出来、放在 guest RAM 里的 64 字节表（画像 `[interrupt] line_map`）。
    /// 不查这张表就会把中断派给邻线的例程：`1↔2` 那一段看着像"差 1"其实是巧合，
    /// 逻辑 6（L1SM）的物理号是 **1**，逻辑 ≥7 又是恒等 —— 只有查表才对。
    fn phys_line(&self, eng: Engine, line: u32) -> u32 {
        match self.profile.interrupt.line_map {
            Some(table) if line < 64 => eng.read_bytes(table + line, 1)[0] as u32,
            _ => line,
        }
    }

    /// 触发一条中断线。返回 false 表示被屏蔽或正处在嵌套上限，调用方可以稍后重试。
    pub fn raise_irq(&mut self, eng: Engine, line: u32) -> bool {
        if self.suppress_irq {
            return false;
        }
        // 掩码位图按物理号索引，所以一进来就先换算
        let line = self.phys_line(eng, line);
        // 分清是被位图挡住还是被 CPU 的 I 位挡住：两者的处置办法完全不同
        if !self.irq.enabled(line) {
            self.stats.irq_blocked_by_mask += 1;
            return false;
        }
        let cpsr_before = eng.cpsr();
        // 调度器还没起来时不投（见 `task_running`）
        if !self.task_running(eng) {
            self.stats.irq_no_task += 1;
            if self.stats.irq_no_task == 1 && self.trace_on {
                println!("[irq] 当前没有任务在跑，中断一律不投（调度器尚未启动）");
            }
            return false;
        }
        // 真机上取中断这件事本身就会把 CPSR.I 置 1，所以 ISR 跑到一半时下一条中断
        // 是投不进去的。唯一的例外是"内核睡眠等待"（见 `asleep`）：真机那一步等价于 WFI，
        // 控制器里已使能并挂起的中断无视屏蔽位就能把 CPU 叫醒。
        //
        // 这里**不能**再要求 `ctx_stack` 为空。ISR 结尾常常是一次任务切换：内核把
        // `LR-4`（也就是我们的哨兵地址）存进 `[0x4000C838]` 再换任务，于是那份被打断的
        // 上下文归内核保管，要等它哪天恢复该任务时才跳回哨兵由我们还回去。
        // 这段时间里栈非空是正常状态，用它当闸门等于把唤醒通道永久堵死。
        if IrqCtrl::cpu_masked(cpsr_before) && !self.asleep(eng) {
            self.stats.irq_blocked_by_cpu += 1;
            return false;
        }
        if self.ctx_stack.len() >= MAX_NESTING {
            self.stats.irq_nested_full += 1;
            if self.stats.irq_nested_full == 1 {
                // 只报一次：到了这个份上后面每条都会被丢，刷屏没有意义
                println!("[irq] 中断嵌套到顶（{MAX_NESTING} 层）后还在投，说明前面某次 ISR 没走到哨兵返回；后续同类注入一律丢弃");
            }
            return false;
        }
        let ctx = Ctx::capture(eng);
        // 内核保存"中断返回到哪"用的是单槽位 `0x4000C838`（IRQ 入口里 `sub r3, lr, 4` 之后
        // 直接 `str r3, [r0]`）。所以从睡眠里被叫醒时，上一份还没恢复的上下文已经被内核覆盖掉，
        // 永远不可能再跳回我们的哨兵 —— 留着它只会让栈涨满、后面所有唤醒都被嵌套上限挡掉。
        if self.asleep(eng) {
            self.ctx_stack.clear();
        }
        // 硬件在压栈那一刻把"被打断处的 CPSR"整份存进 SPSR，然后自己把 I 位置 1。
        // 这两步都不能省：这份固件的 IRQ 入口（flash 0x18 -> [0x38] = 0x4000A290）
        // 第一条指令就是
        //     MRS R1, SPSR / TST R1, #0x80 / LDM SP!, {R1} / SUBSNE PC, LR, #4
        // 也就是"被打断时中断是关着的那就就地异常返回"。Unicorn 的 SPSR 是一整份
        // 扁平寄存器（`load_cpu_field(spsr)`，见 translate.c），不写它就等于让固件的
        // 这道闸去读上一次的残留值，走哪条分支完全看运气。
        let mut spsr = ctx.cpsr;
        if IrqCtrl::cpu_masked(spsr) {
            // 只有睡眠唤醒会走到这里：真机上等价于 WFI 被叫醒，此时被打断上下文的
            // "有效屏蔽状态"就是那个还没被恢复的 0xC0，清掉 I 位才穿得过上面那道闸。
            spsr &= !IRQ_DISABLE_BIT;
        }
        if self.trace_on {
            let slot = self.profile.interrupt.current_task.map_or(0, |s| eng.u32(s));
            let tcb = if slot == 0 { 0 } else { eng.u32(slot) };
            println!(
                "[irq] 注入线 {line}，中断前 PC={:#x} SP={:#x} CPSR={:#x} SPSR={:#x} 当前任务={:#x}",
                ctx.pc, ctx.sp, ctx.cpsr, spsr, tcb
            );
        }
        self.ctx_stack.push(ctx);
        // 状态寄存器必须由中断控制器自己记账：固件在 ISR 入口读它取中断源，
        // 只写后备存储的话读回来永远是 0，会被判成未知中断源
        self.irq.trigger(line);

        // Unicorn 不导出银行寄存器，所以只能"先切模式、再写寄存器"：
        // 切到 IRQ 模式之后，SPSR / LR / SP 这三个 id 指向的就是 IRQ 银行那一组。
        // 少了这一步，固件 ISR 序言里的 MRS R14,SPSR_irq / STMFD SP! 拿到的全是旧值，
        // 返回时自然回不到哨兵地址。
        // 异常入口一律是 ARM 态，所以进向量前必须清掉 T 位
        const THUMB_BIT: u32 = 1 << 5;
        let entry = self.profile.interrupt.handler.unwrap_or(self.profile.interrupt.vector);
        // 和真机一致：进 ISR 期间屏蔽后续中断，哨兵返回时由 `Ctx::restore` 还原
        let mut cpsr = (ctx.cpsr & !THUMB_BIT) | IRQ_DISABLE_BIT;
        if entry & 1 != 0 {
            cpsr |= THUMB_BIT;
        }
        if self.mode_switch {
            cpsr = (cpsr & !0x1f) | MODE_IRQ;
            eng.set_cpsr(cpsr);
            eng.set_reg(R::SPSR, spsr);
            eng.set_reg(R::LR, STUB_IRQ_RETURN);
            eng.set_reg(R::SP, ctx.sp);
        } else {
            // 偷懒做法（C 版同款）：不切模式，但 I 位和 SPSR 照样按硬件置。
            // 分发入口 0x4000A290 不读 IRQ 银行组，所以少了银行寄存器也能走通。
            eng.set_cpsr(cpsr);
            eng.set_reg(R::SPSR, spsr);
            eng.set_lr(STUB_IRQ_RETURN);
        }
        eng.set_pc(entry & !1);
        self.stats.irq_ok += 1;
        true
    }

    /// 通过中断方式执行一个固件回调：先把 LR 指到另一个哨兵，跑完自动回到原处。
    ///
    /// `func` 按 C 版的写法带 Thumb 位（如 `0x0816_D9F1`）。四个入参必须在
    /// [`Ctx::capture`] **之后**再写，否则会被当成"被打断时的值"，回调返回时一并恢复掉。
    pub fn start_callback(&mut self, eng: Engine, func: u32, args: [u32; 4]) {
        let ctx = Ctx::capture(eng);
        self.callback_ctx = Some(ctx);
        for (reg, value) in [R::R0, R::R1, R::R2, R::R3].into_iter().zip(args) {
            eng.set_reg(reg, value);
        }
        eng.set_lr(STUB_CALLBACK_RETURN);
        eng.set_pc(func);
    }

    pub(crate) fn on_stub_reached(&mut self, eng: Engine, addr: u32) {
        // 每个哨兵对应"LR + 实际落点"两个地址，见 `config::STUB_IRQ_LAND` 的说明
        if (STUB_IRQ_LAND..=STUB_IRQ_RETURN).contains(&addr) {
            match self.ctx_stack.pop() {
                Some(ctx) => {
                    self.stats.irq_returned += 1;
                    if self.trace_on {
                        println!(
                            "[irq] 中断返回，恢复 PC={:#x} SP={:#x}，剩余嵌套={}",
                            ctx.pc,
                            ctx.sp,
                            self.ctx_stack.len()
                        );
                    }
                    ctx.restore(eng);
                }
                None => {
                    self.stats.irq_orphan_return += 1;
                    println!("[irq] 中断返回栈空，忽略 {addr:#x}");
                }
            }
        } else if (STUB_CB_LAND..=STUB_CALLBACK_RETURN).contains(&addr) {
            match self.callback_ctx.take() {
                Some(ctx) => ctx.restore(eng),
                None => println!("[irq] 回调上下文缺失，忽略 {addr:#x}"),
            }
        } else {
            println!("[irq] 落到未知哨兵 {addr:#x}，PC={:#x}", eng.pc());
        }
    }

    /// SIM 卡侧的中断线：0 号卡是 5，1 号卡是 28
    pub const SIM_IRQ_LINES: [u32; 2] = [irq_line::SIM1, irq_line::SIM2];
}
