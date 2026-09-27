use std::path::{Path, PathBuf};

/// 画像缺失时给出的提示：这些是芯片级参数，与固件无关
pub const KB: u32 = 1024;
pub const MB: u32 = 1024 * KB;



/// 执行流接管哨兵页：把 LR 写成这里的地址，固件"返回"时由块 hook 还原上下文。
/// 页内填充 `b .`，万一哨兵失效就原地停住而不是跑飞。
///
/// 注意 ARM 态 ISR 的尾声常写成 `SUBS PC, R14, #4`（补偿预取偏移），于是 LR 写 +4
/// 实际落到 +0；回调那条链同理。所以落点是"一对地址"，与 C 版
/// `uc_hook_add(..., CPU_ISR_CB_ADDRESS, CPU_ISR_CB_ADDRESS + 4)` 的区间挂法一致。
pub const STUB_BASE: u32 = 0x5000_0000;
pub const STUB_SIZE: u32 = MB;
/// 中断返回 LR（+4）与实际落点（+0，被 ISR 的 `SUBS PC, LR, #4` 减掉）
pub const STUB_IRQ_LAND: u32 = STUB_BASE;
pub const STUB_IRQ_RETURN: u32 = STUB_BASE + 0x04;
/// 回调返回 LR（+0xC）与实际落点（+8）
pub const STUB_CB_LAND: u32 = STUB_BASE + 0x08;
pub const STUB_CALLBACK_RETURN: u32 = STUB_BASE + 0x0C;


/// 各类伪中断的注入周期
pub const RTC_TICK_MS: u64 = 500;
/// 主循环空转步长，保证哨兵失效时仍能响应退出
/// 一个时间片：跑这么多 guest 指令就回到主循环注入事件、刷新屏幕。
///
/// **必须按指令数截止，不能按宿主墙钟**：`emu_start` 的 timeout 是墙钟，每片切在哪条指令上
/// 随宿主负载变。实测（同一份二进制、同样 18 秒、不加任何观察钩子）两次运行的固件轨迹
/// 就不一致：中断注入时机不同、`谁在挂超时` 一次 28 次 15、创建任务个数 46/47/48 都在跳。
/// 那样任何"改一处代码、看轨迹变化"的判断全部失效。
///
/// **数值取多少是"C 版等效比值"的问题，不是"真芯片吞吐"的问题。** 每片代表 2 ms 仿真时间
/// （见 `EMU_SLICE_US`，13 MHz 计数器按它推进），所以 N/2ms 就是模型假设的 CPU 吞吐：
///   - N = 250 000 ≈ 208 MHz ARM9 的真实吞吐，物理忠实，但实测 18 秒只走到 57 片就再也推不动
///     （本机 Unicorn 约 0.8 M 指令/秒），等一个 tick 要跑几分钟，测不了。
///   - N = 15 000 → 和 C 版"每 5 ms 宿主墙钟投一次 tick"的等效比值一致（C 版正是靠这个比值
///     把开机跑完的），18 秒约 940 片、tick 每约 1.9 万条指令一次。
/// `MT6252_SLICE_INSNS` 可以覆盖它，用来扫这个比值。
pub const EMU_SLICE_INSNS: usize = 15_000;

/// 一个时间片代表的仿真时间（微秒），只用于换算 13 MHz 计数器和 TDMA
pub const EMU_SLICE_US: u64 = 2_000;

/// 把毫秒换算成时间片数。
///
/// 所有周期性的事件注入都必须走这个换算，**不要**用 `Instant` 比墙钟：宿主每片耗时
/// 随负载变（同一个 CTIRQ1 使能窗口能差 7 倍），按墙钟注入就让两次同样的运行走出不同
/// 的固件轨迹，"改一处代码、看轨迹变化"这类判断全部失效。13 MHz 计数器已经按片推进了
/// （见 `Vm::clock13`），RTC/SIM/周期 tick 是剩下的三个墙钟源。
pub const fn slices_per(ms: u64) -> u64 {
    (ms * 1000 + EMU_SLICE_US - 1) / EMU_SLICE_US
}

/// RTC 秒节拍的片数周期（SIM 卡事件不在这里排周期：见 `Vm::pump_sim`，它每片都试投）
pub const RTC_TICK_SLICES: u64 = slices_per(RTC_TICK_MS);

/// 开关型环境变量：**只有 `=1` 算开**。文档里这些变量一律写成 `MT6252_XXX=1`，而
/// `env::var(..).is_ok()` 会把 `=0`、`=off`、甚至设成空串都当成"开"，等于
/// `set MT6252_TRACE=0` 反而打开了 trace。
pub fn env_on(key: &str) -> bool {
    std::env::var(key).is_ok_and(|v| v.trim() == "1")
}

fn project_root() -> PathBuf {
    let exe = std::env::current_exe().unwrap_or_else(|_| PathBuf::from("."));
    exe.parent().map(Path::to_path_buf).unwrap_or_else(|| PathBuf::from("."))
}

/// 依次尝试：环境变量 `MT6252_ROM_DIR`、exe 旁的 `Rom/`、以及向上各级目录里
/// C 版工程的 `MT6252_Simulator/bin/Rom`。这样从 `target/debug` 直接跑也不用拷文件。
pub fn asset(name: &str) -> PathBuf {
    if let Ok(dir) = std::env::var("MT6252_ROM_DIR") {
        let p = PathBuf::from(dir).join(name);
        if p.exists() {
            return p;
        }
    }
    let exe_dir = project_root();
    let mut candidates: Vec<PathBuf> = Vec::new();
    for dir in exe_dir.ancestors() {
        candidates.push(dir.join("Rom").join(name));
        candidates.push(dir.join("MT6252_Simulator/bin/Rom").join(name));
        candidates.push(dir.join("MT6252_Simulator_Release/Rom").join(name));
    }
    candidates.push(PathBuf::from("Rom").join(name));
    candidates
        .into_iter()
        .find(|p| p.exists())
        .unwrap_or_else(|| exe_dir.join("Rom").join(name))
}

pub fn sdcard_image() -> PathBuf {
    asset("fat32.img")
}
