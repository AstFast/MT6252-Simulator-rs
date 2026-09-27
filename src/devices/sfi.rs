//! SPI Flash 控制器（SFI，0x810A_xxxx）。
//!
//! 固件用它读写 NVRAM。握手方式很朴素：把命令和地址写进 GPRAM 窗口，向 MAC_CTL
//! 写一次"bit2|bit3 都为 1"的值（实测固件写 0xa 再 0xe）启动，然后**反复读 MAC_CTL 等忙位
//! 清 0**。命令在"固件读 MAC_CTL 的那一刻"被模拟执行并回写状态，与 C 版的读钩子时序一致。
//!
//! 命令表与 C 版 `hookRamCallBack` 的 `case RW_SFI_MAC_CTL` 完全一致（01/02/05/06/9F/C0/B9/AF/38），
//! **两边都不实现普通的 read-array（03/13/0B…）**：C 版认不出的命令走空的 `default`，
//! 所以"NVRAM 从 flash 里读"这件事在两个模拟器里其实都没有真发生 —— 开机日志里的
//! FAT/NVRAM 数据全部来自 MSDC 那条 SD 卡路径。真要给 SFI 加读命令，应该从
//! `FLASH_WINDOW | address` 把 `input_len` 字节搬进数据窗口。
//!
//! 实测固件确实会发我们（和 C 版）都不认的命令：寄存器序列显示它在触发前**每次都重新写过命令
//! 头**（`写 0x810a0800 = 5` … 后来 `写 0x810a0800 = 0`），所以"解出旧值"这件事并不存在，
//! `0x00`/`0x50` 是它主动发的真实操作，而 C 版对它们同样走空的 `default` —— 两个模拟器在这里
//! 行为一致，C 版照样跑到了渲染界面，所以 SFI 的未知命令**不是**开机卡住的原因。
//!
//! C 版把整段 0x810A 当**普通 RAM** 映射，只挂观察型读写钩子（`main.c:592-599`），所以它的读回
//! 值来自 guest RAM（固件写 + 设备回填共用一块），另有一份 `SF_C_Frame.cacheData[]` 只记固件的
//! 写、专供解码。既然实测"每次触发前命令头都被重写"，一份窗口就足够等价，不再拆成两份。

use crate::engine::Engine;
use crate::memmap::sfi;

/// GPRAM 数据窗口大小（按 4 字节字计数）。
///
/// **不能是 64**：一次传输 = 4 字节头（`opcode + 24 位地址`）+ 最多一整页 256 字节数据
/// = 260 字节，实测 `OUTPUT_LEN=0x104`、`chunk = 2×[0xF016B0D0] = 0x100`。64 字只有 256 字节，
/// 于是**页尾那 4 个字节静默掉出窗口**，而驱动的校验恰好只看页尾那一个字节
/// （RAM 副本 `0x8CA: ldrb r1,[r5]` 读 `0x0800_0000 + addr + chunk - 1` 与源缓冲比，
/// 不等就返 -1）⇒ 表现成"同一条 page-program 原地重发上万次"。
/// 取 66 字（264 字节）刚好装下整页并留 4 字节余量，不多占后面的寄存器槽。
const GPRAM_WORDS: usize = 66;

/// 诊断环形缓冲的深度：只保留最近这么多次寄存器访问
const HISTORY: usize = 24;

// 命令字，取自 C 版 `main.c` 里 `case RW_SFI_MAC_CTL` 各分支的注释名
const CMD_WRITE_SR: u32 = 0x01;
const CMD_PAGE_PROG: u32 = 0x02;
const CMD_READ_SR: u32 = 0x05;
const CMD_WREN: u32 = 0x06;
const CMD_READ_ID: u32 = 0x9f;
const CMD_SET_BURST: u32 = 0xc0;
const CMD_ENTER_DPD: u32 = 0xb9;
const CMD_READ_ID_QPI: u32 = 0xaf;
const CMD_SST_QPIEN: u32 = 0x38;

/// Flash 窗口映射到目标内存的基址
const FLASH_WINDOW: u32 = 0x0800_0000;

/// 把一次 MAC 读的回答搬进**共享**数据窗口。
///
/// 窗口前 4 字节是这次传输的头（`opcode` + 24 位地址，MSB first），设备回答的字节从第 4 字节
/// 起逐字节覆盖，数量 = `input_len - 4`；头永远不动，超出窗口的尾巴丢掉（窗口只有 256 字节，
/// 与 C 版 `RW_SFI_GPRAM_DATA_REG..+256` 一致）。
///
/// 抽成自由函数只为了让"读到底答了什么"能不起 Unicorn 就单测。
fn fill_window(input_len: u32, bytes: &[u8], window: &mut [u32; GPRAM_WORDS]) {
    if input_len <= 4 {
        return;
    }
    let want = (input_len - 4) as usize;
    for (k, b) in bytes.iter().take(want).enumerate() {
        let i = 1 + k / 4;
        if i >= GPRAM_WORDS {
            break;
        }
        let shift = (k % 4) * 8;
        window[i] = (window[i] & !(0xffu32 << shift)) | u32::from(*b) << shift;
    }
}

#[derive(Debug)]
pub struct SfiFlash {
    mac_ctl: u32,
    /// 命令已被接收，等待固件读 MAC_CTL 时执行
    armed: bool,
    output_len: u32,
    input_len: u32,
    /// 固件与设备**共用**的数据窗口：固件把命令头和写入数据放进这里，设备把读回的数据也
    /// 回写到同一批地址，固件再从这些地方读走 —— 与真机的 FIFO 窗口一致，也和 C 版
    /// "整段当普通 RAM 映射 + 读钩子回填"的效果一致（C 版的 `cacheData[]` 只是它自己
    /// 记的一份写值，不参与读回）。
    gpram: [u32; GPRAM_WORDS],
    status_reg: [u8; 3],
    /// 没认出来的命令次数，只用来限制诊断日志的输出量
    unhandled: u32,
    /// 已执行的命令条数，只用来限制"每条都报"的诊断量（见 `run_command`）
    issued: u32,
    /// 命令游程诊断的上一对 `(命令, 头)` 与连发次数，见 `run_command`
    last_cmd: u32,
    last_head: u32,
    run_len: u32,
    /// 最近一次"把 CON 写成触发值"的 guest PC。命令是从哪条驱动例程发出来的，只能靠它回答：
    /// 静态点名的那批 SFI 例程（`0x086F_27B0` 发命令 / `0x086F_30A8` 触发自旋）**实测整轮 0 命中**，
    /// 也就是说真正在发命令的是另一条路，不记 PC 就找不到它是谁。
    armed_at: u32,
    /// 最近若干次寄存器访问 `(地址, 值, 宽度)`，宽度 0 表示"读"。
    /// 只在打"未处理命令"时才吐出来，用来回答"命令字怎么变成 0 的"。
    history: [(u32, u32, u8); HISTORY],
    hist_pos: usize,
}

impl Default for SfiFlash {
    fn default() -> Self {
        Self {
            mac_ctl: 0,
            armed: false,
            output_len: 0,
            input_len: 0,
            gpram: [0; GPRAM_WORDS],
            status_reg: [0; 3],
            unhandled: 0,
            issued: 0,
            last_cmd: 0,
            last_head: 0,
            run_len: 0,
            armed_at: 0,
            history: [(0, 0, 0); HISTORY],
            hist_pos: 0,
        }
    }
}

impl SfiFlash {
    pub fn read(&mut self, eng: Engine, addr: u32) -> Option<u32> {
        match addr {
            sfi::MAC_CTL => {
                self.remember(addr, self.mac_ctl, 0);
                if self.armed {
                    self.run_command(eng);
                }
                Some(self.mac_ctl)
            }
            sfi::OUTPUT_LEN => Some(self.output_len),
            sfi::INPUT_LEN => Some(self.input_len),
            a if self.in_gpram(a) => {
                let value = self.gpram[self.gpram_index(a)];
                self.remember(addr, value, 0);
                Some(value)
            }
            _ => None,
        }
    }

    /// `len` 是合并成整字之前的原始访问宽度（1/2/4）。
    pub fn write(&mut self, eng: Engine, addr: u32, value: u32, len: usize) {
        self.remember(addr, value, len as u8);
        match addr {
            sfi::MAC_CTL => {
                self.mac_ctl = value;
                // 触发判据与 C 版逐位相同：`(value & 0xc) == 0xc`，即 bit2 和 bit3 同时置 1。
                // 注意这按 `defined.h` 的名字是 SFI_WIP|SFI_EN，不是注释里常写的 TRIG|MAC_EN
                // （那两个是 1 和 2 = 0x3）。实测固件的写法是 0xa 再 0xe，只有 0xe 满足。
                if value & 0xc == 0xc {
                    self.armed = true;
                    self.armed_at = eng.pc() as u32;
                }
            }
            sfi::OUTPUT_LEN => self.output_len = value,
            sfi::INPUT_LEN => self.input_len = value,
            a if self.in_gpram(a) => {
                let i = self.gpram_index(a);
                self.gpram[i] = value;
                let _ = eng;
            }
            _ => {}
        }
    }

    fn in_gpram(&self, addr: u32) -> bool {
        (sfi::GPRAM_DATA..sfi::GPRAM_DATA + (GPRAM_WORDS * 4) as u32).contains(&addr)
    }

    /// 记一笔访问到环形缓冲。宽度 0 是"读"（写只有 1/2/4）。
    fn remember(&mut self, addr: u32, value: u32, len: u8) {
        self.history[self.hist_pos] = (addr, value, len);
        self.hist_pos = (self.hist_pos + 1) % HISTORY;
    }

    /// 按时间顺序吐出最近的访问，用来回答"命令字是怎么变成 0 的"
    fn dump_history(&self) {
        for i in 0..HISTORY {
            let (addr, value, len) = self.history[(self.hist_pos + i) % HISTORY];
            if addr == 0 {
                continue;
            }
            let op = if len == 0 { "读" } else { "写" };
            println!("[sfi]   {addr:#010x} {op} = {value:#010x} ({} 字节)", len.max(1));
        }
    }

    fn gpram_index(&self, addr: u32) -> usize {
        ((addr - sfi::GPRAM_DATA) / 4) as usize
    }

    /// 首字：低 8 位是命令，随后三个字节是 24 位地址（大端）
    fn run_command(&mut self, eng: Engine) {
        self.armed = false;
        let head = self.gpram[0];
        let command = head & 0xff;
        let address = (head >> 24) | (((head >> 16) & 0xff) << 8) | (((head >> 8) & 0xff) << 16);
        match command {
            CMD_PAGE_PROG => {
                // 去掉 1 字节命令 + 3 字节地址，剩下的才是写入数据
                let len = self.output_len.saturating_sub(4) as usize;
                let payload: Vec<u8> = self.gpram[1..1 + len.div_ceil(4).min(GPRAM_WORDS - 1)]
                    .iter()
                    .flat_map(|w| w.to_le_bytes())
                    .take(len)
                    .collect();
                eng.write_bytes(FLASH_WINDOW | address, &payload);
            }
            CMD_READ_SR => {
                let [s0, s1, s2] = self.status_reg;
                self.gpram[0] = u32::from(s0) | u32::from(s1) << 8 | u32::from(s2) << 16;
            }
            CMD_WRITE_SR => self.status_reg = [head as u8, (head >> 8) as u8, (head >> 16) as u8],
            CMD_READ_ID => {
                // C 版返回 1/2/3 三个字节，够固件认出"有芯片"。
                // 不碰 INPUT_LEN：C 版只往数据窗口写这三个字，长度寄存器归固件所有。
                self.gpram[0] = 1;
                self.gpram[1] = 2;
                self.gpram[2] = 3;
            }
            CMD_WREN | CMD_ENTER_DPD | CMD_READ_ID_QPI | CMD_SST_QPIEN | CMD_SET_BURST => {}
            other => {
                // 命令字解错的话，光看 `other` 是猜不出格式的，把整字和长度一起打出来
                if self.unhandled < 6 {
                    let (con, out_len, in_len, g1, g2) =
                        (self.mac_ctl, self.output_len, self.input_len, self.gpram[1], self.gpram[2]);
                    println!(
                        "[sfi] 未处理命令 {other:#04x} head={head:#010x} con={con:#x} out_len={out_len:#x} in_len={in_len:#x} g1={g1:#010x} g2={g2:#010x}"
                    );
                    // 头两次把寄存器序列一起打出来：命令字是从哪一笔写进来的、触发前又读了什么，
                    // 只有按时间顺序的访问序列能回答
                    if self.unhandled < 2 {
                        self.dump_history();
                    }
                }
                self.unhandled += 1;
            }
        }
        // 硬件语义：MAC 读把 flash 阵列的字节搬进**同一个数据窗口**，头 4 字节（opcode + 24 位地址，
        // MSB first —— 由驱动 RAM 副本 `0xD3C: bl 0xC46`(bswap32) 核对）之内不放数据，所以读到的
        // 第一个字节落在 `窗口 + 4`，长度 = `INPUT_LEN - 4`（与写侧"减去 1 命令 + 3 地址"对称）。
        // 实测本固件的读全是 `出0x1/入0x5|0x8|0x104`，即 1/4/256 个数据字节。
        // 不搬这一趟，驱动读回的就是它自己写进去的命令头 —— 子代理逐条核对过：驱动把
        // `0x810A0800` 当**响应寄存器**用，NVRAM 正是拿着"成功 + 全 0 数据"在外层反复重发整条 op。
        if self.input_len > 4 {
            let want = (self.input_len - 4) as usize;
            let room = GPRAM_WORDS * 4 - 4;
            let bytes = eng.read_bytes(FLASH_WINDOW.wrapping_add(address), want.min(room));
            fill_window(self.input_len, &bytes, &mut self.gpram);
        }
        // 按"游程"报命令：驱动的重发环（`0x086F28BA` / `0x086F290E`）测的是数据窗口
        // `0x810A0800` 的 bit8 与 bits[23:16]（不是 CON），所以只有"这条命令回读到什么"能解释
        // 它为什么重发。逐条打会被开机期几百条正常命令淹掉，所以只在 (命令,头) 变化时打一行，
        // 外加每 1024 次重复打一行 —— 卡死的重发环会以"同一命令 连发 N 次"的形式自己跳出来。
        if command != self.last_cmd || head != self.last_head {
            if self.run_len > 1 {
                println!(
                    "[sfi] 上一条 {:#04x} head={:#010x} 连发 {} 次",
                    self.last_cmd, self.last_head, self.run_len
                );
            }
            self.last_cmd = command;
            self.last_head = head;
            self.run_len = 0;
        }
        self.run_len += 1;
        // 周期性**采样**：游程打印只能看出"同一条命令连发"，看不出命令流有没有在往前走。
        // 每 1000 条抽一行，把地址一起打出来 —— 头一遍遍变大就是在读下一页，原地打转才是卡住。
        if self.issued % 1000 == 0 {
            let n = self.issued;
            println!(
                "[sfi] 采样 #{n} 命令 {command:#04x} head={head:#010x} 长度(出{:#x}/入{:#x}) 回读={:#010x} 发起PC={:#010x}",
                self.output_len,
                self.input_len,
                self.gpram[0],
                self.armed_at,
            );
        }
        if self.issued < 400 && self.run_len % 1024 == 1 {
            println!(
                "[sfi] 命令 {command:#04x} head={head:#010x} 长度(出{:#x}/入{:#x}) 连发第{}次 → 回读 gpram[0]={:#010x} bit8={} 高字节={:#04x}",
                self.run_len,
                self.output_len,
                self.input_len,
                self.gpram[0],
                (self.gpram[0] >> 8) & 1,
                (self.gpram[0] >> 16) & 0xff
            );
        }
        self.issued += 1;
        // 命令完成：清掉 TRIG，只留 MAC_EN，固件轮询到此值即退出等待循环
        self.mac_ctl = 2;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 驱动写进窗口的头（`opcode + 24 位地址 MSB first`）在任何回答之后都不许被覆盖：
    /// 状态查询类的下一条命令就是靠回读它自己那 4 字节来判形的
    #[test]
    fn read_data_never_touches_the_four_byte_header() {
        let mut w = [0u32; GPRAM_WORDS];
        w[0] = 0x0b_80_78_02;
        fill_window(0x104, &[0xAA; 8], &mut w);
        assert_eq!(w[0], 0x0b_80_78_02, "头 4 字节必须原样留着");
    }

    /// 回答的第一个字节落在窗口第 4 字节，并**只**改给过值的那些字节。
    /// 窗口是"地址→u32"的小端视图：`w[0]` = 窗口第 0..3 字节（头），`w[1]` = 第 4..7 字节，
    /// 所以第 4 字节是 `w[1] & 0xFF`，不是 `w[1] >> 24`——这条测试故意把原值写成
    /// `0xDE_AD_BE_EF`，让"没给值的第 7 字节"停在最高位字节 `0xDE` 上， endian 搞错就会红。
    #[test]
    fn read_data_starts_at_window_byte_four_and_is_byte_wise() {
        let mut w = [0u32; GPRAM_WORDS];
        w[1] = 0xDE_AD_BE_EF;
        fill_window(4 + 3, &[0x11, 0x22, 0x33], &mut w);
        assert_eq!(w[1] & 0xFF, 0x11, "第 4 字节 = 回答的第一个字节");
        assert_eq!((w[1] >> 8) & 0xFF, 0x22);
        assert_eq!((w[1] >> 16) & 0xFF, 0x33);
        assert_eq!((w[1] >> 24) & 0xFF, 0xDE, "第 7 字节没给值，就该保持原值");
        assert_eq!(w[2], 0, "只给了 3 个字节，第 8 字节那一句都不该动");
    }

    /// `INPUT_LEN - 4` 才是数据字节数：只给 1 个字节的状态回答（0x05 那一族）不进这条通道，
    /// 免得把"读状态"实现成"读阵列"
    #[test]
    fn short_reads_are_left_to_their_own_handlers() {
        let mut w = [0u32; GPRAM_WORDS];
        w[1] = 0x1234_5678;
        fill_window(1, &[0xFF; 16], &mut w);
        fill_window(4, &[0xFF; 16], &mut w);
        assert_eq!(w[1], 0x1234_5678);
    }

    /// 窗口只有 `GPRAM_WORDS*4` 字节，超出部分**丢掉**而不是越界写；头那句永远不动
    #[test]
    fn over_long_read_is_truncated_at_the_window_end() {
        let mut w = [0u32; GPRAM_WORDS];
        w[0] = 0x0b_80_78_02;
        fill_window(4 + GPRAM_WORDS as u32 * 4, &[0x5A; 512], &mut w);
        assert_eq!(w[GPRAM_WORDS - 1], 0x5A_5A_5A_5A, "最后一句填满");
        assert_eq!(w[0], 0x0b_80_78_02, "丢掉尾巴不等于可以动头");
        assert!(w[1..].iter().all(|v| *v == 0x5A_5A_5A_5A));
    }

    /// 一整页（4 字节头 + 256 字节数据 = `OUTPUT_LEN=0x104`）必须**逐个字节**都装得下。
    ///
    /// 钉住的就是那次误判的根：窗口按 64 字（256 字节）开，尾页 4 字节掉出去，
    /// 而驱动只回读**页尾那一个字节**做校验（RAM 副本 `0x8CA`），于是一条 page-program
    /// 被上层原地重发上万次。这里把 256 个不同的字节都点一遍，少一个就红。
    #[test]
    fn window_holds_a_whole_page_of_two_hundred_sixty_bytes() {
        let mut w = [0u32; GPRAM_WORDS];
        w[0] = 0x00_77_00_02;
        let bytes: Vec<u8> = (0..256usize).map(|k| k as u8).collect();
        fill_window(4 + 256, &bytes, &mut w);
        for k in 0..256usize {
            let lane = ((w[1 + k / 4] >> ((k % 4) * 8)) & 0xFF) as u8;
            assert_eq!(lane, bytes[k], "第 {k} 个数据字节被丢了");
        }
        assert_eq!(w[0], 0x00_77_00_02, "头还是那四个字");
    }
}
