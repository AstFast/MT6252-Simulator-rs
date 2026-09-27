//! MMIO 分派：把 Unicorn 的读写回调路由到具体设备。
//!
//! 与 C 版的对应关系：C 版是一个横跨 `0x8000_0000..0xA200_0000` 的
//! `UC_HOOK_MEM_READ/WRITE` 大 switch；这里换成按区间注册的 MMIO 后端，
//! 未命中的偏移落到 [`Vm::reg_store`] 这个稀疏后备存储上，语义等同于
//! "写进假内存、下次读回来"。

use crate::engine::Engine;
use crate::memmap::{self, block, intc, l1_win, msdc_reg, systimer};
use crate::vm::Vm;

/// MSDC 传输完成回调在固件里的地址（C 版硬编码 `0x816D9F0 + 1`）
const MSDC_DONE_CALLBACK: u32 = 0x816D_9F1;

impl Vm {
    pub fn mmio_read(&mut self, eng: Engine, addr: u32, len: usize) -> u32 {
        if self.trace_on {
            *self.poll_counts.entry(addr & !3).or_insert(0) += 1;
        }
        // 与写侧同理：设备按寄存器地址精确匹配，必须给对齐后的字地址，
        // 字节/半字读的偏移由下面的 lane 抽取处理
        let word = addr & !3;
        let value = match self.device_read(eng, word) {
            Some(v) => {
                // 设备伪造出来的返回值也要落进后备存储，之后同一地址的普通读才能看到
                self.reg_store.insert(word, v);
                v
            }
            None => self.load_u32(word),
        };
        if len >= 4 {
            value
        } else {
            value >> ((addr & 3) * 8) & (u32::MAX >> (32 - len * 8))
        }
    }

    pub fn mmio_write(&mut self, eng: Engine, addr: u32, len: usize, value: u32) {
        let word = addr & !3;
        let merged = if len >= 4 {
            value
        } else {
            let shift = (addr & 3) * 8;
            let mask = (u32::MAX >> (32 - len * 8)) << shift;
            (self.load_u32(word) & !mask) | ((value << shift) & mask)
        };
        self.reg_store.insert(word, merged);
        // 必须传对齐后的字地址：设备都是按寄存器地址精确匹配的，
        // 而一次字节/半字写带的是它自己的偏移，直接透传会让整条写被漏掉
        self.device_write(eng, word, len, merged);
    }

    /// 13 MHz 计数器。按**时间片**推进而不是按宿主墙钟现算：TCG 下 guest 跑多久和宿主过多久
    /// 完全不成比例，按墙钟算会让每次运行的时序都不一样（实测同一份代码两次跑，启动深度和
    /// SFI 命令次数都不同），而且"读一次变一次"还会把固件的双读惯用法逼成死循环。
    pub fn tick_13mhz(&self) -> u32 {
        self.clock13
    }

    /// 补丁和开机预置用的统一写入口：MMIO 区间走后备存储（直接写 guest 会递归触发
    /// 自己的写回调），普通内存才真的写进 Unicorn
    pub fn prime_write(&mut self, eng: Engine, addr: u32, value: u32, size: u8) {
        if memmap::is_mmio(addr) {
            self.store_u32(addr, value);
            // 光写后备存储不够：设备自己拥有的寄存器（C1XX 的比较值、中断控制器的掩码等）
            // 是从 `device_write` 记账的，绕过去的话画像里 prime 的值设备根本看不见
            self.device_write(eng, addr & !3, 4, value);
            return;
        }
        match size {
            1 => eng.set_u8(addr, value as u8),
            2 => eng.set_u16(addr, value as u16),
            8 => eng.write_bytes(addr, &value.to_le_bytes()),
            _ => eng.set_u32(addr, value),
        }
    }

    pub fn store_u32(&mut self, addr: u32, value: u32) {
        self.reg_store.insert(addr & !3, value);
    }

    fn load_u32(&self, addr: u32) -> u32 {
        self.reg_store.get(&(addr & !3)).copied().unwrap_or(0)
    }

    fn device_read(&mut self, eng: Engine, addr: u32) -> Option<u32> {
        // C1XX 的 13 MHz 自由运行计数器：芯片真行为，固件用它算延时和超时，
        // 停在 0 会让"等 N 个 tick"的轮询循环永远出不来。
        // 读出去的值要过一层快照，否则固件 `0x0816f72c` 那种"连读两次取相等"的安全读会原地打转
        if addr == systimer::TICK {
            return Some(self.tick_13mhz());
        }
        // TDMA 帧号同理：固件"等下一帧"的循环必须看到它递增才退得出
        if addr == systimer::TDMA_FRAME {
            return Some(self.c1xx.next_frame());
        }
        // 画像里登记的"伪装寄存器"：C 版在读 hook 里回写固定值，用来跳出固件的等待循环
        if let Some(value) = self.read_override.get(&(addr & !3)) {
            return Some(*value);
        }
        match addr >> 16 {
            block::INTC => {
                let v = self.irq.read(addr);
                // LISR 派发器（`0x0800_2E38`）就是靠这一次读决定"这次服务哪个中断"：
                // `index = [0x810100D8] & 0x3F`，bit8 置位就直接返回。所以这一条读就是
                // "投递到底算不算数"的判定点 —— 不打它，只能对着"中断返回得很快"猜。
                if self.trace_on && addr == intc::INT_STATUS {
                    println!("[intc] 读 INT_STATUS = {v:?} PC={:#010x}", eng.pc());
                }
                v
            }
            block::LCD => self.lcd.read(addr),
            block::L1_MAILBOX => self.l1.read(addr, eng.pc()),
            block::SFI => self.sfi.read(eng, addr),
            block::KPD => self.keypad.read(addr),
            block::RTC => self.rtc.read(addr),
            block::MSDC => self.sd.read(addr),
            block::SIM1 | block::SIM2 => {
                let i = usize::from(addr >> 16 == block::SIM2);
                let v = self.sim[i].read(addr);
                // 每一次 SIM 寄存器读都要打：设备内部的 `read()` 是静默的，之前"ATR 回不去"
                // 的判断只能靠 `[sim] TIDE/中断使能` 两条写日志推断，读走几个字节、读到什么
                // 值完全看不见 —— 而看不见和"没发生"在日志里长得一模一样。
                if self.trace_on {
                    println!("[sim{i}] 读 {addr:#010x} = {v:?} PC={:#010x}", eng.pc());
                }
                v
            }
            _ => None,
        }
    }

    /// `len` 是这次访问的宽度（合并成整字之前的原始宽度）。设备需要它来分辨"固件按字节
    /// 写数据窗口"这种情况 —— C 版把 GPRAM 当普通 RAM 挂观察钩子，它的命令解码用的是
    /// **未合并**的原始值，我们这边传进来的已经是合并后的整字。
    fn device_write(&mut self, eng: Engine, addr: u32, len: usize, value: u32) {
        match addr >> 16 {
            block::INTC => {
                self.irq.write(addr, value);
                if self.trace_on {
                    self.irq.log_change(addr, value);
                }
            }
            block::DMA => {
                // 每一条 DMA 寄存器写都要打：`[dma] 通道启动` 只在 `START == 0x8000` 时打印，
                // 所以"驱动没启动 DMA"和"驱动启动了但值不是 0x8000 / 写到别的偏移"在日志里
                // 长得一模一样。第三条块读就是这么卡住的（`dump/sd13fix.log` 片 483）。
                if self.trace_on {
                    println!(
                        "[dma] 写 {addr:#010x} = {value:#010x} 宽{len} PC={:#010x} 片={}",
                        eng.pc(),
                        self.stats.slices
                    );
                }
                self.dma.write(addr, value);
                // 这里**不**补做 MSDC 搬运：搬运只由 CMD 触发（见 `msdc_command`）。
                // 早先在通道启动的瞬间抢先做一次，等于用上一条命令的 ARG 填满刚刚
                // 为下一条命令武装的缓冲区，实测把 LBA 0 的 MBR 写进了 LBA 2048
                // 的引导扇区缓冲 `0x6420` ⇒ `BytsPerSec=0` ⇒ −34 卸载。
            }
            block::KPD => self.keypad.write(addr, value),
            block::RTC => self.rtc.write(addr, value),
            block::SIM1 | block::SIM2 => {
                if self.trace_on {
                    println!(
                        "[sim{}] 写 {addr:#010x} = {value:#010x} 宽{len} PC={:#010x}",
                        usize::from(addr >> 16 == block::SIM2),
                        eng.pc()
                    );
                }
                self.sim[usize::from(addr >> 16 == block::SIM2)].write(addr, value);
            }
            block::LCD => {
                // 每一条 LCD 写都要打，**包括 `+0xC`（帧传送/kick）**：早先这里把 kick
                // 过滤掉了，结果"固件从来没踢过帧"这条结论根本没法被推翻 —— 它是从一条
                // 刻意不打印的日志里读出来的。现在连宽度一起打，因为 `write_reg` 拿到的是
                // 合并后的整字，而固件对 `+0xC` 用的是 `strh`（半字），两者不是一回事。
                if self.trace_on {
                    println!(
                        "[lcd] 写 {addr:#010x} = {value:#010x} 宽{len} PC={:#010x}",
                        eng.pc()
                    );
                }
                self.lcd.write(eng, addr, value);
            }
            block::SFI => self.sfi.write(eng, addr, value, len),
            block::MSDC => {
                // 逐条打出 CMD/ARG 的写入：挂载被 −34 卸载的根因是"补做传输用的 ARG 不是
                // 这条命令的参数"，而 CMD、ARG、DMA 启动三者的先后**只能实测**（静态从发送器
                // 反汇编得到的"先 ARG 后 CMD"与两种取值时机都对不上）。
                if self.trace_on
                    && (addr == msdc_reg::CMD || addr == msdc_reg::ARG)
                {
                    println!(
                        "[msdc] 写 {addr:#010x} = {value:#010x} PC={:#010x} 片={}",
                        eng.pc(),
                        self.stats.slices
                    );
                }
                self.sd.write(addr, value);
                if addr == msdc_reg::CMD {
                    self.msdc_command(eng);
                }
            }
            b if b == block::SYS_TIMER => {
                if addr == systimer::TDMA_FNIT {
                    self.c1xx.rearm_tdma();
                }
                let raw = self.tick_13mhz();
                self.c1xx.write(addr, value, raw);
                if self.trace_on {
                    println!("[c1xx] 写 {addr:#010x} = {value:#010x}");
                }
            }
            // L1 信箱窗口：设备只读，写仍然只落进 `reg_store`。这里把写打出来，
            // 用来判定那些状态字到底是 CPU 写的握手、还是只能由 L1 侧提供。
            // 只在值变化时打：门铃每拍都踢，全打会刷出几十万行
            block::L1_MAILBOX => {
                if self.trace_on && self.l1_written.get(&addr) != Some(&value) {
                    println!("[l1win] 写 {addr:#010x} = {value:#010x}");
                    self.l1_written.insert(addr, value);
                }
                self.l1.write(addr, value);
                if addr == l1_win::DOORBELL && value & 1 != 0 {
                    self.l1_service(eng);
                }
            }
            _ => {}
        }
    }

    /// L1 侧被门铃叫醒、取走 AP 排的请求之后的动作：把挂起位图清掉。
    ///
    /// 真机上 L1 是总线主设备，能直接写 AP 的 INIT_SRAM；ROM 里没有对应的清位代码
    /// （全 flash 搜 `0x4000C820`/`0x4000C818` 基址字面量，只有置位的 `orr` 和开机
    /// 那一次 memset），所以这一步只能由 L1 模型做。位图为 0 时不动作、也不打印。
    fn l1_service(&mut self, eng: Engine) {
        let Some(mask) = self.profile.l1.pending_mask else { return };
        let pending = eng.u32(mask);
        if pending == 0 {
            return;
        }
        println!("[l1] 应答挂起请求 {pending:#010x}");
        self.prime_write(eng, mask, 0, 4);
    }

    /// 把一个模型值推进信箱页。设 `l1_pushing` 闸是为了防递归；闸有没有真的挡到东西，
    /// 由 `l1_api_reentry` 计数在收尾时报告，不靠推理。
    fn push_l1(&mut self, eng: Engine, addr: u32, value: u32) {
        self.l1_pushes += 1;
        self.l1_pushing = true;
        eng.set_u32(addr, value);
        self.l1_pushing = false;
    }

    /// **模型拥有**的信箱字（= 真机上由 L1 这一侧总线主设备写的字）。
    /// 固件往这些口写东西不该改变下一次读的结果，所以写钩子要把它改回来。
    /// `REQUEST` 和门铃不在表里 —— 那两个是 AP 拥有的，固件写完就该留在页里。
    fn l1_model_word(&self, word: u32) -> Option<u32> {
        match word {
            l1_win::COMMAND => Some(self.l1.command()),
            l1_win::STATUS => Some(self.l1.status()),
            l1_win::RESPONSE => Some(self.l1.response()),
            l1_win::READY_A => Some(l1_win::READY_A_VALUE),
            l1_win::READY_B => Some(l1_win::READY_B_VALUE),
            l1_win::STATE => Some(l1_win::STATE_VALUE),
            _ => None,
        }
    }

    /// 开机时按模型值预置信箱页。
    ///
    /// 这一步是"改造前后语义一致"的关键：页变成真内存之后，未预置的字节全是 0，
    /// 而旧的读回调对 `COMMAND` 恒回 `0x3703`、对 13 MHz 链的三个字恒回 `8/0x80/0xF807`。
    /// 不预置就等于把 13 MHz 服务链的第 2、3、4 关和命令字的第 5 关全改成"必然失败"，
    /// 而失败是**静默**的（那些检查过了才 `bl`，没过就直接 bail）。
    pub fn prime_l1_page(&mut self, eng: Engine) {
        let words: Vec<(u32, u32)> = (memmap::L1_PAGE_BASE..memmap::L1_PAGE_BASE + memmap::L1_PAGE_SIZE)
            .step_by(4)
            .filter_map(|w| self.l1_model_word(w).map(|v| (w, v)))
            .collect();
        for (addr, value) in &words {
            self.push_l1(eng, *addr, *value);
        }
        let line: Vec<String> = words.iter().map(|(a, v)| format!("{a:#010x}={v:#010x}")).collect();
        println!("[l1page] 信箱页改成真内存，已按模型值预置 {}", line.join(" "));
    }

    /// 信箱页的**写**钩子（这一页只有写钩子，读完全不经过宿主 —— 改造的全部目的）。
    ///
    /// 钩子给的是"这次写了什么"（值 + 宽度），而**页里回读到的可能是写之前的旧值**：
    /// RAM 写钩子和 store 落地的先后没有保证，所以判断"固件刚写了什么"一律用钩子参数按宽度
    /// 合并，不用回读（`l1page` 那行一次性对照就是这件事的实测记录）。
    ///
    /// 要做两件事：
    /// 1. 按写事件推进模型：`REQUEST.go` 的**边沿**决定 L1 收没收下命令、事务有没有结束；
    ///    `COMMAND` 每写一次出一次应答；门铃 bit0 叫醒 L1 去清挂起位图；
    /// 2. 把模型拥有的字改回模型值。不改的话固件 `strh 0x3783` 之后，睡眠检查再读同一个字
    ///    会读到它自己写进去的值，而旧的读回调恒回 `0x3703` —— 语义就悄悄变了，而且是往
    ///    "必然触发 trace point `0x172` → 嵌套异常关机"那一侧变。
    pub fn l1_page_write(&mut self, eng: Engine, addr: u32, len: usize, value: u32) {
        if self.l1_pushing {
            self.l1_api_reentry += 1;
            return;
        }
        let word = addr & !3;
        self.l1_writes += 1;
        // **不能回读页里的现值来决定"固件刚写了什么"**：Unicorn 的 RAM 写钩子和这次 store
        // 的落地先后没有保证。合并规则和 `mmio_write` 完全一样（按访问宽度拼进整字），
        // 所以钩子早到、晚到都得到同一个"写完之后该有什么"。
        let (shift, mask) = if len >= 4 {
            (0, u32::MAX)
        } else {
            let shift = (addr & 3) * 8;
            (shift, (u32::MAX >> (32 - len * 8)) << shift)
        };
        let ram = eng.u32(word);
        let merged = (ram & !mask) | (value << shift & mask);
        if word == l1_win::REQUEST && !self.l1_order_logged {
            self.l1_order_logged = true;
            // 一次性对照：钩子给的值 vs 页里的现值。两者不等就说明钩子是在 store **之前**
            // 回调的（这条是给未来的我看的，别当成推理）
            println!("[l1page] 首见写 REQUEST：钩子值={value:#x} 宽{len} 页内现值={ram:#x} 合并={merged:#x} PC={:#010x}", eng.pc());
        }
        let value = merged;
        if self.trace_on && self.l1_written.get(&word) != Some(&value) {
            println!("[l1win] 写 {word:#010x} = {value:#010x} 宽{len} PC={:#010x}", eng.pc());
            self.l1_written.insert(word, value);
        }
        match word {
            l1_win::REQUEST => {
                let go = value & l1_win::REQUEST_GO != 0;
                if go != self.l1_request_go {
                    self.l1_request_go = go;
                    self.l1.write(l1_win::REQUEST, u32::from(go));
                    let status = self.l1.status();
                    self.push_l1(eng, l1_win::STATUS, status);
                    if !go {
                        // 事务收尾。旧读回调在这里把 `transactions` 加 1，命令字跟着从
                        // `0x3703` 变 0；应答字也一起撤掉，等下一条命令重新置
                        let (response, command) = (self.l1.response(), self.l1.command());
                        self.push_l1(eng, l1_win::RESPONSE, response);
                        self.push_l1(eng, l1_win::COMMAND, command);
                    }
                }
            }
            l1_win::COMMAND => {
                // 设备自己按命令出应答（`L1Mailbox::write` 里的 `answer_of`），这里只负责
                // 把应答字推进页里；命令字本身由收尾那一段改回模型值
                self.l1.write(l1_win::COMMAND, value);
                let response = self.l1.response();
                self.push_l1(eng, l1_win::RESPONSE, response);
            }
            l1_win::DOORBELL => {
                if value & 1 != 0 {
                    self.l1_service(eng);
                }
            }
            _ => {}
        }
        // 统一收尾：模型拥有的字被改写的话按模型值推回去
        if let Some(model) = self.l1_model_word(word) {
            if value != model {
                self.push_l1(eng, word, model);
            }
        }
    }

    /// CMD17/18（读卡）与 CMD24/25（写卡）配合 DMA 通道做一次数据搬运。
    ///
    /// 命令按**索引**认（`cmd & 0x3F`），不按整字相等：固件写进 `0x810E0024` 的是
    /// `池常量 | [state+4]` 的打包值（`0x0816_0280` 那条就是 `0x0891` = CMD17），
    /// 修饰位一变，精确匹配会**静默**漏掉整条命令。索引式取法对这些常量逐个自洽：
    /// `0x438c&0x3f=12`(CMD12)、`0x008d&0x3f=13`(CMD13)、`0x01a9&0x3f=41`(CMD41)。
    ///
    /// 只认「DMA 先武装、CMD 后到」这一条序。`dump/sddma.log`：8 次通道启动、9 次 CMD17，
    /// 6 对当场搬走，3 次到达时未武装（片 431、482 的 `ARG=0` 是驱动重发的，483 的
    /// `ARG=0x100000` 就是被"抢先补做"坑掉的 LBA 2048）。删掉那套机制后用 `MT6252_TRACE=1`
    /// 复跑一整条导航（`dump/sdtrace.log`，19500 片）：**成功搬运 352 次、到达时未武装只有
    /// 1 次、传输失败 0 次**，那唯一一次正是片 431 的 `ARG=0`，驱动在片 482 重发并落在武装
    /// 之后 ⇒ 没有丢读。真实硬件在未武装时 FIFO 无人取数，本来也搬不成，所以不欠账、不挂起。
    fn msdc_command(&mut self, eng: Engine) {
        let raw = self.sd.cmd();
        let index = raw & 0x3F;
        if !matches!(index, 17 | 18 | 24 | 25) {
            return;
        }
        let arg = self.sd.arg();
        if !self.msdc_transfer(eng, raw, index, arg) && self.trace_on {
            println!("[msdc] cmd={raw:#06x} 索引={index} arg={arg:#x} 到达时 DMA 未武装 → 本次无搬运");
        }
    }

    /// 真搬一次数据。返回 `false` = DMA 通道还没武装，调用方按「不欠账」处理。
    fn msdc_transfer(&mut self, eng: Engine, raw: u32, index: u32, arg: u32) -> bool {
        use crate::devices::dma::ChannelId;
        let dma = self.dma.channel(ChannelId::Msdc);
        if !dma.configured {
            return false;
        }
        let (count, dst) = (dma.transfer_count as usize, dma.data_addr);
        let ok = if index == 17 || index == 18 {
            match self.sd.read_at(arg, count) {
                Some(data) => {
                    eng.write_bytes(dst, &data);
                    true
                }
                None => false,
            }
        } else {
            let data = eng.read_bytes(dst, count);
            self.sd.write_at(arg, &data)
        };
        if !ok {
            println!("[msdc] 传输失败 cmd={raw:#06x} arg={arg:#x} len={count}");
        } else if self.trace_on {
            // 成功也要打：删掉"抢先补做"那套之后，"日志里没有报错行"已经读不出"数据真的落进
            // 了这块缓冲区"这件事——而配对错一位的表现恰恰就是没有报错行。
            println!("[msdc] 搬运 索引={index} arg={arg:#x} → {count} 字节 @ {dst:#x}");
        }
        let dma = self.dma.channel_mut(ChannelId::Msdc);
        dma.configured = false;
        let notify = dma.int_enable;
        dma.int_enable = false;
        self.sd.finish_transfer();
        if notify {
            self.start_callback(eng, MSDC_DONE_CALLBACK, [0, 0, 0, 0]);
        }
        true
    }

    /// SIM 卡 DMA 的收尾：把缓冲区内容和命令交回设备状态机
    pub(crate) fn finish_sim_dma(&mut self, eng: Engine, which: u8) {
        let ch = if which == 0 { crate::devices::dma::ChannelId::Sim1 } else { crate::devices::dma::ChannelId::Sim2 };
        let dma = self.dma.channel(ch);
        if !dma.configured {
            return;
        }
        let (addr, len, to_card) = (dma.data_addr, dma.transfer_count, dma.direction == crate::devices::dma::Direction::RamToReg);
        self.dma.channel_mut(ch).configured = false;
        let card = &mut self.sim[which as usize];
        if to_card {
            card.on_tx_finished(eng, addr, len);
        } else {
            card.on_rx_finished(eng, addr, len);
        }
        // 与 `Vm::pump_sim` 同一套规矩（那边修过；这份是它的重复实现，之前漏改）：
        // 1) 投递前把这次的通道位写进 `0x8109_0014`，固件的 SIM LISR 第一件事就是读它，
        //    读到 0 会当伪中断立刻返回；
        // 2) **只有投出去了才清挂起位** —— 先清后投的话，撞上 CPSR.I 或线屏蔽就把这次
        //    卡事件永久丢掉，ATR 再也回不去（C 版是 `if (!StartInterrupt(5)) EnqueueVMEvent(…)`）。
        let (irq_channel, due) = (card.irq_channel, card.irq_pending && (card.irq_enable & card.irq_channel) != 0);
        if due {
            // 逐条下标访问、不长期持有借用：`raise_irq` 要 `&mut self`。
            self.sim[which as usize].irq_status |= irq_channel;
            let line = Vm::SIM_IRQ_LINES[which as usize];
            if self.raise_irq(eng, line) {
                self.sim[which as usize].irq_pending = false;
            }
        }
    }
}
