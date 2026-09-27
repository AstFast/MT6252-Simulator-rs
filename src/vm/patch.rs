//! 按固件画像里的补丁表打补丁。
//!
//! C 版给整个 ROM 区间挂了一个 `UC_HOOK_CODE`，于是**每一条指令**都要回回调走一遍 switch；
//! 这里按地址逐点注册。**但"逐点"不等于"只有命中的那几条有开销"**：实测 0 个探针 60 秒跑
//! 87458 片、挂满 410 个探针只跑 266 片（慢 330 倍），因为 Unicorn 让**每个基本块都扫一遍
//! 钩子表**。所以探针要按需用 `MT6252_PROBES` 筛（见 `filter_probes`）。动作本身也是数据
//! （见 `profile.rs` 的 `Action`），换固件改 TOML，不改这个文件。

use unicorn_engine::Unicorn;

use crate::engine::Engine;
use crate::profile::Action;
use crate::vm::Vm;

fn add_hook(emu: &mut Unicorn<'static, Vm>, addr: u32, actions: Vec<Action>) -> Result<(), String> {
    emu.add_code_hook(addr as u64, addr as u64, move |emu, _, _| {
        let eng = unsafe { Engine::from_raw(emu.get_handle()) };
        for action in &actions {
            emu.get_data_mut().apply_patch(action, eng);
        }
    })
    .map_err(|e| format!("给 {addr:#x} 挂补丁失败: {e}"))?;
    Ok(())
}

/// 同一份代码的另一个可执行地址。镜像被原样复制两遍时（见 `profile::mirror_span`），
/// `base+off` 与 `base+off+span` 是同一颗指令，而固件里的指针表两类目标都有。
fn twin(base: u32, size: u32, span: Option<u32>, addr: u32) -> Option<u32> {
    let span = span?;
    if addr < base || addr >= base + size {
        return None;
    }
    let off = addr - base;
    Some(if off < span { addr + span } else { addr - span })
}

/// 这个动作是不是"只看不改"的探针（探针都有 `label`，功能型动作没有）。
fn is_probe(a: &Action) -> bool {
    matches!(
        a,
        Action::Count { .. }
            | Action::LogStr { .. }
            | Action::LogStrAt { .. }
            | Action::LogReg { .. }
            | Action::LogPair { .. }
            | Action::LogAt { .. }
    )
}

fn label_of(a: &Action) -> Option<&str> {
    match a {
        Action::Count { label }
        | Action::LogStr { label, .. }
        | Action::LogStrAt { label, .. }
        | Action::LogReg { label, .. }
        | Action::LogPair { label, .. }
        | Action::LogAt { label, .. } => Some(label),
        _ => None,
    }
}

/// 只保留**功能型**动作 + 标签命中 `MT6252_PROBES` 的探针。
///
/// 为什么必须能筛：实测同一台机器、同样 60 秒，画像里 0 个探针跑 **87458 片**
/// （≈21.9 M 指令/秒，TCG 的正常水平），挂满 410 个探针只跑 **266 片**（≈0.066 M/秒）——
/// **慢 330 倍**。原因在 Unicorn：code hook 不是"命中那条指令才有开销"，而是**每个基本块
/// 都要扫一遍钩子表**，所以钩子数量直接乘在解释成本上。本文件开头那句"按地址逐点注册，
/// 只有命中的那几条指令有开销"是错的，实测已推翻。
///
/// 取值：`none` 一个探针都不装；逗号分隔的串按**标签子串**匹配；没设这个环境变量就全装
/// （保持旧行为，方便复现历史日志）。功能型动作（`set_reg`/`poke*`/`copy_reg`）永远装,
/// 因为它们改的是固件行为，不是观察。
fn filter_probes(specs: Vec<crate::profile::PatchSpec>) -> (Vec<crate::profile::PatchSpec>, String) {
    let want = match std::env::var("MT6252_PROBES") {
        Err(_) => return (specs, String::from("MT6252_PROBES 未设：探针全装")),
        Ok(v) => v,
    };
    let keys: Vec<String> = want
        .split(',')
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .collect();
    let none = keys.iter().any(|k| k == "none");
    let out: Vec<crate::profile::PatchSpec> = specs
        .into_iter()
        .filter(|p| {
            !is_probe(&p.action)
                || (!none
                    && label_of(&p.action).is_some_and(|l| keys.iter().any(|k| l.contains(k.as_str()))))
        })
        .collect();
    let kept_probes = out.iter().filter(|p| is_probe(&p.action)).count();
    (
        out,
        format!("MT6252_PROBES={want}：保留 {kept_probes} 个探针（none=一个都不装）"),
    )
}

/// 标签里不许有空格。`[count]` 一行是 `标签=计数` 用空格拼起来的，标签带空格就会被
/// 切成两个 token：`USIM_Lisr 入口=1` 存成键 `入口`，按原名查永远是"不存在" ——
/// 一个**每加一个带空格的标签就多造一个假零**的机制（画像里曾同时存在 17 个这样的标签，
/// 包括 `offer API:*`、`CTIRQ1 LISR`、`cold reset 打印点`、`lcd_fb_update 调用点1(L1D)`）。
/// 与其靠约定，不如在装载时把空格换成 `_` 并报到控制台。
fn sanitize_labels(a: &mut Action) -> Option<(String, String)> {
    let label = match a {
        Action::Count { label }
        | Action::LogStr { label, .. }
        | Action::LogStrAt { label, .. }
        | Action::LogReg { label, .. }
        | Action::LogPair { label, .. }
        | Action::LogAt { label, .. } => label,
        _ => return None,
    };
    if !label.contains(' ') {
        return None;
    }
    let fixed = label.replace(' ', "_");
    let old = std::mem::replace(label, fixed.clone());
    Some((old, fixed))
}

pub fn install(emu: &mut Unicorn<'static, Vm>) -> Result<(), String> {
    let (mut specs, how) = filter_probes(emu.get_data().profile.patch.clone());
    let mut renamed: Vec<(String, String)> = Vec::new();
    for s in specs.iter_mut() {
        if let Some(pair) = sanitize_labels(&mut s.action) {
            renamed.push(pair);
        }
    }
    for (old, fixed) in &renamed {
        println!("[patch] 标签里的空格已换成下划线：\"{old}\" -> {fixed}（带空格的标签会被 [count] 行的空格分隔切碎，按原名查必得假零）");
    }
    // 记下"这一轮到底装了哪些 `count` 探针标签"，`print_counters` 才能在 `MT6252_ZEROS=1` 时把
    // 没命中的也列成 `标签=0`。只收 `Count`：`log_*` 动作不往 `counters` 里记数，列成 0 会是假的。
    // 不记就只能从"缺席"推断"没命中"，而那两件事在旧版报告里长得一样。
    let mut labels: Vec<String> = specs
        .iter()
        .filter(|s| is_probe(&s.action))
        .filter_map(|s| match &s.action {
            Action::Count { label } => Some(label.clone()),
            _ => None,
        })
        .collect();
    labels.sort();
    labels.dedup();
    emu.get_data_mut().probe_labels = labels;
    let (base, size, span) = {
        let rom = &emu.get_data().profile.rom;
        (rom.base, rom.size, rom.alias_span)
    };
    let explicit: std::collections::HashSet<u32> = specs.iter().map(|s| s.addr).collect();
    // 一个地址只允许有一个 Unicorn 钩子：挂第二个会把第一个**静默吞掉**，被吞的那个恒 0，
    // 而恒 0 和"固件没走到这里"长得一模一样。所以先按地址归并，一个钩子里按画像顺序依次
    // 执行全部动作 —— 同一地址既能计数又能打寄存器，不必二选一。
    let mut order: Vec<u32> = Vec::new();
    let mut group: std::collections::HashMap<u32, Vec<Action>> = std::collections::HashMap::new();
    let mut mirrors = 0usize;
    for spec in &specs {
        group.entry(spec.addr).or_insert_with(|| {
            order.push(spec.addr);
            Vec::new()
        }).push(spec.action.clone());
        // 画像里已经显式挂了另一份别名的就不自动补，否则动作会被算两遍
        if let Some(other) = twin(base, size, span, spec.addr).filter(|t| !explicit.contains(t)) {
            group.entry(other).or_insert_with(|| {
                order.push(other);
                mirrors += 1;
                Vec::new()
            }).push(spec.action.clone());
        }
    }
    for addr in &order {
        add_hook(emu, *addr, group[addr].clone())?;
    }
    println!("[patch] {how}");
    println!(
        "[patch] 已挂 {} 个固件补丁点{}，共 {} 处",
        specs.len(),
        if mirrors == 0 {
            String::new()
        } else {
            format!("（另加 {mirrors} 个镜像别名：这份 ROM 是同一镜像复制两遍，只钉一个别名会把\"跑的是另一份\"看成\"从没执行\"）")
        },
        order.len()
    );
    // 归并是静默的，所以把"一个地址上有多个动作"报出来：执行顺序就是画像里的书写顺序，
    // 后面若有 `set_reg` 会改掉前面动作要读的寄存器，这条就是唯一的线索。
    let mut multi: Vec<u32> = order
        .iter()
        .filter(|addr| group[addr].len() > 1)
        .copied()
        .collect();
    multi.sort_unstable();
    for addr in multi {
        let names: Vec<String> = group[&addr].iter().map(|a| a.to_string()).collect();
        println!("[patch] {addr:#010x} 合并 {} 个动作，按画像顺序执行：{}", names.len(), names.join(" ｜ "));
    }
    if crate::config::env_on("MT6252_LIST_PATCHES") {
        for spec in &specs {
            println!("  {:#010x}  {}{}", spec.addr, spec.action, pad_note(&spec.note));
        }
    }
    Ok(())
}

impl Vm {
    fn apply_patch(&mut self, action: &Action, eng: Engine) {
        match action {
            Action::SetReg { reg, value } => eng.set_reg(reg.id(), *value),
            Action::CopyReg { reg, src } => {
                let value = eng.reg(src.id());
                eng.set_reg(reg.id(), value);
            }
            Action::Poke { target, value, size } => self.prime_write(eng, *target, *value, *size),
            Action::PokeFromReg { target, reg } => {
                let value = eng.reg(reg.id());
                self.prime_write(eng, *target, value, 4);
            }
            Action::PokeAtReg { reg, offset, value, size } => {
                let addr = eng.reg(reg.id()) + offset;
                self.prime_write(eng, addr, *value, *size);
            }
            Action::LogStr { reg, label } => {
                let text = guest_cstr(eng, eng.reg(reg.id()));
                println!("[fw] {label}: {text}");
            }
            Action::LogStrAt { target, label } => println!("[fw] {label}: {}", guest_cstr(eng, *target)),
            Action::LogReg { reg, label } => println!("[fw] {label}({:#x})", eng.reg(reg.id())),
            Action::LogPair { reg, reg2, label } => {
                println!("[fw] {label}({:#x})({:#x})", eng.reg(reg.id()), eng.reg(reg2.id()))
            }
            Action::LogAt { reg, offset, label, size } => {
                let base = eng.reg(reg.id()) as u32;
                // 走 try_read：探针读的是固件给的指针，野值时**不能**把整个仿真打崩
                match eng.try_read(base.wrapping_add(*offset), *size as usize) {
                    Some(b) => {
                        let v = b.iter().rev().fold(0u64, |a, &x| (a << 8) | u64::from(x));
                        println!("[fw] {label}([{base:#x}] + {offset:#x}) = {v:#x}");
                    }
                    None => println!("[fw] {label}([{base:#x}] + {offset:#x}) = <读不到>"),
                }
            }
            Action::Count { label } => *self.counters.entry(label.clone()).or_insert(0) += 1,
        }
    }
}

fn pad_note(note: &str) -> String {
    if note.is_empty() {
        String::new()
    } else {
        format!("  — {note}")
    }
}

/// 读 Guest 内存里的 C 字符串，最多 128 字节，非 UTF-8 的部分替换掉。
/// 走 `try_read`：这几个钩子打的是**固件给的指针**（`kal_debug_print(r0)` 之类），
/// 野值时 `read_bytes` 里的 `debug_assert_eq!` 会把整个 debug 构建打崩。
fn guest_cstr(eng: Engine, addr: u32) -> String {
    let Some(raw) = eng.try_read(addr, 128) else {
        return String::from("<读不到>");
    };
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    String::from_utf8_lossy(&raw[..end]).into_owned()
}
