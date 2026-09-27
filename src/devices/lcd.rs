//! LCD 显示控制器（0x9000_xxxx）：四个叠加图层，RGB565。
//!
//! 寄存器取自数据手册（`MT6252_Simulator/docs/MT6252_GSM_GPRS_Baseband_Processor_Data_Sheet_v1.0.pdf`
//! 的 LCD 章：页 323 起的地址表 + 页 363 起的字段定义）。每层一个 0x30 字节的块，层 0 块基址
//! `0x9000_00B0`，手册末尾明确写了"层 1~3 的每个寄存器字段都与层 0 相同"：
//!
//! ```text
//! +0x00 LCD_LxWINCON   层控制：CLRFMT 色彩格式（001=RGB565）、SRC_KEYEN/DST_KEYEN 色键使能、
//!                      ROTATE、ALPHA/ALPHA_EN、BYTE_SWP、DITHER_EN、SCRL_EN
//! +0x04 LCD_LxWINKEY   色键 CLRKEY[31:0]（RGB565 只用低 16 位）
//! +0x08 LCD_LxWINOFS   这一层贴到屏幕上的位置：Y-OFFSET[31:16] / X-OFFSET[15:0]，单位像素
//! +0x0C LCD_LxWINADD   显存起始字节地址（手册：RGB565 要 2 字节对齐）
//! +0x10 LCD_LxWINSIZE  这一层自己的尺寸：ROW[31:16] / COLUMN[15:0]，单位像素
//! +0x14 LCD_LxWINSCRL  滚动起始偏移
//! +0x18 LCD_LxWINMOFS  显存取图偏移
//! +0x1C LCD_LxWINPITCH 显存每行字节数（手册：图像总宽 × 每像素字节数）
//! ```
//!
//! 与 [`crate::memmap::lcd_reg`] 的四个地址寄存器对得上：`0xBC / 0xEC / 0x11C / 0x14C`
//! 正好是 `0xB0 + n*0x30 + 0x0C`，所以"块基址 0xB0、步长 0x30"这个划分与现有常量互洽
//! （[`tests::layer_block_layout_matches_memmap_constants`] 把这条钉住了）。
//!
//! 建模了：`WINADD`（地址）、`WINKEY`（色键）、`WINOFS`（位置）、`WINSIZE`（尺寸）、
//! `WINPITCH`（跨距）。没建模的是 `WINCON`（色彩格式/色键使能/字节序/旋转/alpha）、
//! `WINSCRL`、`WINMOFS`，原因写在 [`Lcd::write_reg`] 的 TODO 里。

use crate::engine::Engine;
use crate::memmap::lcd_reg;

/// 固件没编程 LxWINKEY 时用的色键：RGB565 的纯蓝。
/// 这是 C 版的实测值（`main.c` 的 `if (color != 0x1f)` 才覆盖下层），不是数据手册的复位值——
/// 手册没给复位值。固件真写 LxWINKEY 之后以写进来的值为准（见 [`Layer::key`]）。
const DEFAULT_KEY: u16 = 0x001f;

/// 层 0 寄存器块基址与层间步长，单位字节
const LAYER_BASE: u32 = 0x9000_00B0;
const LAYER_STRIDE: u32 = 0x30;

/// 层内寄存器偏移
const OFF_WINCON: u32 = 0x00;
const OFF_WINKEY: u32 = 0x04;
const OFF_WINOFS: u32 = 0x08;
/// `LxWINOFS` 的**零偏**：寄存器里存的不是像素坐标，而是"坐标 + 1024"。
///
/// 依据不是手册措辞，而是固件自己怎么算这个值（反汇编逐条核过）：
/// ```text
/// 0x0830A104  movs  r4, #1 ; lsls r4, r4, #0xa     ; r4 = 1024
/// 0x0830A10E  adds  r0, r3, r4 ; strh r0, [r5, #8] ; X-OFFSET = 请求的 x + 1024
/// 0x0830A112  adds  r0, r4     ; strh r0, [r5, #0xa] ; Y-OFFSET = 请求的 y + 1024
/// ```
/// 本固件实测把 L0 放在请求位置 (0,0)，于是寄存器读回来是 `1024,1024`。
/// 把它当无符号坐标用 ⇒ `x >= 屏宽` ⇒ **整层被合成器跳过，画面必然全黑**；
/// 而 C 版参考**根本不读 WINOFS**（只解析 4 个 WINADD 和 `0x9000000C`），所以它总能贴上去
/// —— 这是一处两版行为不等价，不是硬件差异。
const WINOFS_BIAS: i32 = 1024;
const OFF_WINADD: u32 = 0x0C;
const OFF_WINSIZE: u32 = 0x10;
const OFF_WINSCRL: u32 = 0x14;
const OFF_WINMOFS: u32 = 0x18;
const OFF_WINPITCH: u32 = 0x1C;

/// 每像素字节数。CLRFMT 还支持 8bpp 索引 / RGB888 / YUYV422，本模型只按 RGB565 解
const BPP: usize = 2;

/// 一个图层。"没写过寄存器"与"写了 0"在硬件上未必等价，所以几何参数用 0 表示"未编程"，
/// 兜底策略见 [`Lcd::blit`]。
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Layer {
    /// LxWINADD：显存起始地址，直接当 guest 物理地址用。0 = 未编程 = 这层不参与合成
    addr: u32,
    /// LxWINKEY：本层色键；没写过则用 [`DEFAULT_KEY`]
    key: Option<u16>,
    /// LxWINOFS：这层在屏幕上的位置，单位像素。
    /// **已减去 [`WINOFS_BIAS`]，是有符号的**：固件往这个寄存器里写的是"请求位置 + 1024"，
    /// 直接当无符号像素坐标用会把整层判成在屏幕外。
    x: i32,
    y: i32,
    /// LxWINSIZE 的 COLUMN / ROW：这层自己有多宽多高，0 = 未编程
    width: u32,
    height: u32,
    /// LxWINPITCH：显存每行字节数，0 = 未编程（按本层宽度 × BPP 推）
    pitch: u32,
}

#[derive(Debug, Default)]
pub struct Lcd {
    /// 索引越小越靠底层：合成时按下标正序遍历，L0 先画、L3 最后画，最后画的盖在上面
    layers: [Layer; 4],
    /// 帧传输寄存器写 0，表示一帧画完需要重绘
    pub dirty: bool,
    /// 分辨率来自固件画像
    pub width: u32,
    pub height: u32,
}

impl Lcd {
    pub fn new(width: u32, height: u32) -> Self {
        Self { width, height, ..Self::default() }
    }
}

impl Lcd {
    pub fn write(&mut self, _eng: Engine, addr: u32, value: u32) {
        self.write_reg(addr, value);
    }

    /// 诊断用：把四层的编程状态压成一行。合成出来全黑时，第一个要排除的就是
    /// "寄存器根本没被写过 ⇒ 每层 `addr=0` ⇒ `blit` 原地返回"，而这件事光看像素分不出来。
    pub fn layer_state_line(&self) -> String {
        self.layers
            .iter()
            .enumerate()
            .map(|(i, l)| {
                format!(
                    "L{i} 地址={:#010x} {}x{}@{},{} 跨距={}",
                    l.addr, l.width, l.height, l.x, l.y, l.pitch
                )
            })
            .collect::<Vec<_>>()
            .join(" | ")
    }

    /// LCD 控制器的读侧。目前只有一个位要答。
    ///
    /// `0x9000_0000` 是 `LCD_CON`，**bit0 = `LCD_ON`**。全 ROM 对 `+0` 零写点，所以这一位只能
    /// 由硬件给，而两个读者要求相反（逐条反汇编核过，别再看旧笔记）：
    /// - `0x0839_AF00` 的软件复位握收 `while ((u16)[0x90000000] & 1) ;` —— 要它**为 0**；
    /// - `0x0813_1E3A` 的 GDI 提交门 —— bit0 **为 1 就直接提交**，为 0 才去问
    ///   `[0xF01D2A5C] ∈ {0x17,0x18}`，两条都不满足就 return 0 拒绝提交图层。
    ///
    /// ⇒ 恒答 1 会把握收成死循环（实测吃掉片 2830 之后的全部深度）；恒答 0 满足握收，但图层
    /// 提交只能等状态字节自己走到 0x17/0x18 —— 实测它停在 `2 → 6 → 2`，所以画面全黑。
    /// 与 C 版一致（C 把这段映射成普通 RAM、对 `0x90000000` 一个钩子都没有，固件没写过 ⇒ 读回 0）。
    /// **别把它当"正在传帧"**：帧传送寄存器是 `+0xC`，与此无关。
    pub fn read(&self, addr: u32) -> Option<u32> {
        match addr & !3 {
            lcd_reg::DISP_STAT => Some(0),
            _ => None,
        }
    }

    /// 寄存器记账。与 [`Lcd::write`] 分开是为了不起仿真进程也能单测
    /// （`write` 只多了一个目前用不到的 `Engine` 形参）。
    fn write_reg(&mut self, addr: u32, value: u32) {
        // `Vm::device_write` 传的是"实际访问地址"，字节/半字访问时低位可能非 0；
        // 它已经把子字写合并成整字了，所以这里对齐一次才不会漏掉这类写
        let addr = addr & !3;
        if addr == lcd_reg::FRAME_TRANSFER {
            // 用 |=：这一帧里已经挂着重绘时，后面写个非 0 不该把它取消掉
            self.dirty |= value == 0;
            return;
        }
        // TODO(需要固件真实写日志才能定): LxWINCON 没解，它是三件事的前提——
        //   1) CLRFMT 决定每像素字节数（现在恒按 RGB565 = 2 字节）；
        //   2) SRC_KEYEN / DST_KEYEN 决定色键到底生效不生效（现在恒生效，与 C 版一致）；
        //   3) BYTE_SWP 会改变像素字节序（现在恒小端，与 ARM 上按 u16 读显存一致）。
        // 这些字段的**位号**在这份 PDF 的位段表里排版塌了（pdftotext 的 -layout 和 -raw
        // 两种模式给出的列都对不齐），所以不猜；等固件真往 0x9000_00B0 写值再说。
        //
        // 2026-09-26 更新：**"等固件写真值"这个前提已经满足了**，整块 dump 见
        // `dump/900000b0_a0_rlayers_s14000.bin`（`MT6252_FB_PNG` 那轮可复现）：
        //   L0 CON=0x00100000 KEY=0x00000000 OFS=0x0400_0400 ADD=0x00035AC0 SIZE=0x0140_00F0 PITCH=0x1E0
        //   L1 CON=0x00104000 KEY=0x0000001F OFS=0x0400_0400 ADD=0x00010280 SIZE=0x0140_00F0 PITCH=0x1E0
        //   L2/L3 全 0
        // 两层的 CON 只差 bit14，而 L1 的 KEY 恰好就是 [`DEFAULT_KEY`] 那个 RGB565 纯蓝，
        // 所以"恒生效"这条在实测帧上与硬件不冲突（L0 的 KEY=0 只把壁纸里的纯黑透掉，
        // 而 fb 初值也是黑，看不出差别）。位号仍未解：pypdf 抽页 368 的位段表，
        // `SRC`/`SRC_KEYEN`/`ROTATE`/`ALPHA_EN`/`ALPHA` 挤在同一行、`Type` 行的 5 个 R/W
        // 对不上 6 个名字，硬猜会把色键门控判反。等出现"该透不透"的帧再解。
        let Some(offset) = addr.checked_sub(LAYER_BASE) else { return };
        let index = (offset / LAYER_STRIDE) as usize;
        if index >= self.layers.len() {
            return;
        }
        let layer = &mut self.layers[index];
        match offset % LAYER_STRIDE {
            OFF_WINADD => layer.addr = value,
            OFF_WINKEY => layer.key = Some(value as u16),
            OFF_WINOFS => {
                // 低半字 X、高半字 Y，**都要减去 [`WINOFS_BIAS`]** 才是像素坐标
                layer.x = (value & 0xffff) as i32 - WINOFS_BIAS;
                layer.y = (value >> 16) as i32 - WINOFS_BIAS;
            }
            OFF_WINSIZE => {
                layer.width = value & 0xffff; // COLUMN
                layer.height = value >> 16; // ROW
            }
            OFF_WINPITCH => layer.pitch = value & 0xffff,
            OFF_WINCON | OFF_WINSCRL | OFF_WINMOFS => {}
            _ => {}
        }
    }

    /// 把四层合成到 `fb`（0x00RRGGBB 布局）。返回是否有实际更新。
    pub fn composite(&mut self, eng: Engine, fb: &mut [u32]) -> bool {
        self.composite_with(&mut |addr, len| eng.read_bytes(addr, len), fb)
    }

    /// [`Lcd::composite`] 的可注入版本：`read(addr, len)` 负责取显存字节。
    /// 抽出来只为了单测能喂假显存——`Engine` 必须有活的 Unicorn 句柄，而这条链路
    /// 到现在为止一颗真实像素都没跑过。生产路径仍走 `composite()`，对外行为不变。
    ///
    /// 注意：这里**不清空** `fb`，所有层都是色键的像素会保留上一帧的内容（与 C 版
    /// 一致）。真机上那种像素落在背景层 L0 上，该显示什么由 L0 自己保证；等固件真的
    /// 开始画屏、能证实 L0 确实铺满全屏且不透明之后再决定要不要每帧先清。
    pub fn composite_with(
        &mut self,
        read: &mut dyn FnMut(u32, usize) -> Vec<u8>,
        fb: &mut [u32],
    ) -> bool {
        if !self.dirty {
            return false;
        }
        let pixels = self.width as usize * self.height as usize;
        if fb.len() < pixels {
            // 这里**不能**清 dirty：清了等于把这帧丢掉，调用方下次看到 dirty=false 就
            // 不再进来，画面永远停在旧内容上
            return false;
        }
        self.dirty = false;
        for layer in self.layers {
            self.blit(layer, read, fb);
        }
        true
    }

    /// 把一层贴到整屏 `fb` 上。按下标正序调用，所以后调用的层在上面。
    fn blit(&self, layer: Layer, read: &mut dyn FnMut(u32, usize) -> Vec<u8>, fb: &mut [u32]) {
        if layer.addr == 0 {
            return; // 这层没配地址
        }
        let screen_w = self.width as usize;
        let screen_h = self.height as usize;
        if layer.x >= screen_w as i32 || layer.y >= screen_h as i32 {
            return; // 整层在屏幕右下之外：一个字节都不该去读（读越界地址在 Unicorn 那边是 assert）
        }
        // TODO(需要固件真实数据才能定): 未编程时的兜底。
        // LxWINSIZE / LxWINPITCH 没写过就按"整屏大小、紧密排列"处理，等于沿用 C 版
        // `renderGdiBufferToWindow()` 的简化（它对每层都读满 屏宽×屏高×2 再逐像素覆盖）。
        // 硬件上真值只能来自固件对这些寄存器的写入，而目前实测固件一条 `[lcd]` 写日志都
        // 没产生过，所以"每层都是全屏同尺寸"既没被证实也没被证伪。这个兜底只给
        // "固件只写了 WINADD"的场合用，不是硬件行为。
        let width = if layer.width == 0 { screen_w } else { layer.width as usize };
        let height = if layer.height == 0 { screen_h } else { layer.height as usize };
        // 跨距缺省按**本层**宽度推，不是按屏幕宽度：窗口比屏幕窄时两者不一样
        let pitch = if layer.pitch == 0 { width * BPP } else { layer.pitch as usize };
        // 负偏移 = 这层从屏幕左/上之外开始画：可见部分要把源像素先跳过，硬件就是这么裁的
        let (skip_col, dst_x) = if layer.x < 0 { (((-layer.x) as usize).min(width), 0usize) } else { (0, layer.x as usize) };
        let (skip_row, dst_y) = if layer.y < 0 { (((-layer.y) as usize).min(height), 0usize) } else { (0, layer.y as usize) };
        // 超出屏幕的部分硬件会裁掉
        let cols = (width - skip_col).min(screen_w - dst_x);
        let rows = (height - skip_row).min(screen_h - dst_y);
        if cols == 0 || rows == 0 {
            return; // 裁完什么都不剩
        }
        // 只读到真正用得上的最后一个像素：多读一整行可能越过 Unicorn 的映射尾
        let need = (skip_row + rows - 1) * pitch + (skip_col + cols) * BPP;
        let raw = read(layer.addr, need);
        let key = layer.key.unwrap_or(DEFAULT_KEY);
        'rows: for row in 0..rows {
            for col in 0..cols {
                let at = (skip_row + row) * pitch + (skip_col + col) * BPP;
                // 注入的 read 给短了（或 guest 读失败）就收工，别让切片 panic
                let Some(px) = raw.get(at..at + BPP) else { break 'rows };
                // 小端：ARM 是小端，显存里的 u16 低字节在前
                let color = u16::from_le_bytes([px[0], px[1]]);
                if color == key {
                    continue; // 色键：透过去，露出下面那层
                }
                fb[(dst_y + row) * screen_w + dst_x + col] = rgb565_to_argb(color);
            }
        }
    }
}

#[inline]
fn rgb565_to_argb(color: u16) -> u32 {
    let r = (color >> 11) as u32;
    let g = (color >> 5 & 0x3f) as u32;
    let b = (color & 0x1f) as u32;
    // 高位补足：5/6/5 位扩展到 8 位
    ((r << 3) | (r >> 2)) << 16 | ((g << 2) | (g >> 4)) << 8 | ((b << 3) | (b >> 2))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 假显存基址。落在 `memmap::CHIP_REGIONS` 的 SDRAM 段（0x0000_0000..8MB 是 Ram），
    /// 这样"图层地址直接当 guest 物理地址用"这件事是按同一个约定测的
    const RAM: u32 = 0x0001_0000;
    const RAM_SIZE: usize = 0x4000;
    /// 四个层各占 4KB，互不重叠
    const L0: u32 = RAM;
    const L1: u32 = RAM + 0x1000;
    const L2: u32 = RAM + 0x2000;
    const L3: u32 = RAM + 0x3000;
    /// 小屏：够测位置和裁剪，又能一眼手数像素下标
    const W: u32 = 4;
    const H: u32 = 3;
    const N: usize = (W * H) as usize;

    const RED: u16 = 0xF800;
    const GREEN: u16 = 0x07E0;
    const BLUE: u16 = 0x001F; // 恰好等于默认色键
    const WHITE: u16 = 0xFFFF;
    /// 一眼认得出的"垃圾"像素：任何越出窗口的读取都会把它漏到屏幕上
    const JUNK: u16 = 0xAAAA;

    /// 按固件的写法组一个 `LxWINOFS`：寄存器存的是"请求坐标 + [`WINOFS_BIAS`]"，
    /// 所以测试里要写偏过的值，而不是裸坐标 —— 否则测的就不是固件真正会写进去的东西。
    fn ofs(x: i32, y: i32) -> u32 {
        let pack = |v: i32| ((v + WINOFS_BIAS) as u32) & 0xffff;
        (pack(y) << 16) | pack(x)
    }
    /// 没画过的 fb 用这个哨兵填，好区分"透过去了"和"画成黑了"
    const SENTINEL: u32 = 0xAABB_CCDD;

    struct FakeVram {
        bytes: Vec<u8>,
        /// 每次读的 (地址, 长度) 记账：用来断言"该跳过的层不碰内存""读的长度不多不少"
        reads: Vec<(u32, usize)>,
    }

    impl FakeVram {
        fn new() -> Self {
            Self { bytes: vec![JUNK as u8; RAM_SIZE], reads: Vec::new() }
        }

        /// 铺一行行像素：`rows[r]` 是第 r 条扫描线，相邻两条线之间隔 `pitch` 字节
        /// （pitch 大于该行实际字节数时，多出来的填充位保持原样，模拟"图比窗口宽"）
        fn put_rows(&mut self, addr: u32, pitch: usize, rows: &[&[u16]]) {
            let base = (addr - RAM) as usize;
            for (r, row) in rows.iter().enumerate() {
                for (c, px) in row.iter().enumerate() {
                    let at = base + r * pitch + c * BPP;
                    self.bytes[at..at + BPP].copy_from_slice(&px.to_le_bytes());
                }
            }
        }

        /// 紧密排列地铺 `count` 个像素
        fn put_solid(&mut self, addr: u32, count: usize, color: u16) {
            let base = (addr - RAM) as usize;
            for i in 0..count {
                let at = base + i * BPP;
                self.bytes[at..at + BPP].copy_from_slice(&color.to_le_bytes());
            }
        }

        /// 整屏一个颜色，再把若干处按线性像素下标覆写
        fn put_screen(&mut self, addr: u32, base: u16, over: &[(usize, u16)]) {
            let mut px = [base; N];
            for (i, v) in over {
                px[*i] = *v;
            }
            let stride = W as usize * BPP;
            self.put_rows(addr, stride, &[&px[0..4], &px[4..8], &px[8..12]]);
        }

        fn read(&mut self, addr: u32, len: usize) -> Vec<u8> {
            self.reads.push((addr, len));
            let base = (addr - RAM) as usize;
            self.bytes[base..base + len].to_vec()
        }
    }

    /// 层 n 的第 off 号寄存器绝对地址
    fn reg(layer: usize, off: u32) -> u32 {
        LAYER_BASE + layer as u32 * LAYER_STRIDE + off
    }

    fn new_lcd() -> Lcd {
        Lcd::new(W, H)
    }

    /// 只配各层的 WINADD（= C 版那种"固件只写地址"的最省事件）并请求重绘。
    /// `None` 表示这层固件根本没碰过。
    fn arm_addr(lcd: &mut Lcd, addrs: &[Option<u32>]) {
        for (i, addr) in addrs.iter().enumerate() {
            if let Some(a) = addr {
                lcd.write_reg(reg(i, OFF_WINADD), *a);
            }
        }
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 0);
        assert!(lcd.dirty, "帧传输写 0 之后应该挂着待重绘");
    }

    fn comp(lcd: &mut Lcd, vram: &mut FakeVram, fb: &mut [u32]) -> bool {
        lcd.composite_with(&mut |a, n| vram.read(a, n), fb)
    }

    fn blank_fb() -> Vec<u32> {
        vec![SENTINEL; N]
    }

    /// 稳定的"位置可辨"颜色：r/g/b 三个字段各占一处，所以 10x10 内互不相同；
    /// 蓝场永远 <= 9，所以不会撞上色键 0x001F
    fn color_of(row: usize, col: usize) -> u16 {
        0x0020 | ((row as u16) << 11) | col as u16
    }

    // ---------- 位扩展 ----------

    /// RGB565 → 8888 必须是"高位复制补到低位"而不是简单左移：5 位全 1 要正好得到 255。
    /// C 版的 `PIXEL565R(v) = ((v>>11)<<3) & 0xff` 就是错的，白色会只到 0xF8
    #[test]
    fn rgb565_expansion_is_bit_replication() {
        for (color, expect) in [
            (0x0000u16, 0x0000_0000u32),
            (RED, 0x00FF_0000), // 纯红
            (GREEN, 0x0000_FF00), // 纯绿
            (BLUE, 0x0000_00FF), // 纯蓝，也正好是色键值
            (WHITE, 0x00FF_FFFF),
            (0xF81F, 0x00FF_00FF), // 红+蓝
            (0x07FF, 0x0000_FFFF), // 绿+蓝
            (0x8000, 0x0084_0000), // 红色最低位 → 16，不是 128
            (0x0001, 0x0000_0008), // 蓝色最低位
            (0x001E, 0x0000_00F7), // 蓝 30/31
            (0x1234, 0x0010_45A5),
            (0xAAAA, 0x00AD_5552),
            (0x5A5A, 0x005A_49D6),
        ] {
            assert_eq!(rgb565_to_argb(color), expect, "{color:#06x} 扩展错了");
        }
    }

    /// 穷举 65536 个值：三个通道都不许溢出到第 4 字节（fb 的约定是 0x00RRGGBB，
    /// softbuffer 那边 alpha 恒 0），且各通道单调、满量程正好 255
    #[test]
    fn rgb565_expansion_is_monotone_and_saturates() {
        for color in 0..=0xFFFFu32 {
            let argb = rgb565_to_argb(color as u16);
            assert_eq!(argb & 0xFF00_0000, 0, "{color:#06x} 污染了 alpha 字节: {argb:#010x}");
        }
        for r in 1..32u32 {
            assert!(
                rgb565_to_argb((r << 11) as u16) > rgb565_to_argb(((r - 1) << 11) as u16),
                "红色通道不单调 @{r}"
            );
            assert_eq!(rgb565_to_argb((r << 11) as u16) >> 16, (r << 3) | (r >> 2));
        }
        for g in 1..64u32 {
            assert_eq!(rgb565_to_argb((g << 5) as u16) >> 8 & 0xFF, (g << 2) | (g >> 4));
        }
        assert_eq!(rgb565_to_argb(0xF800) >> 16, 255, "5 位满量程必须是 255");
        assert_eq!(rgb565_to_argb(0x07E0) >> 8 & 0xFF, 255, "6 位满量程必须是 255");
        assert_eq!(rgb565_to_argb(0x001F) & 0xFF, 255, "5 位满量程必须是 255");
    }

    // ---------- 单层 ----------

    /// 只有一层纯色时整屏每个像素都该是那个 ARGB；顺便钉住"WINPITCH/WINSIZE 未编程时
    /// 一次读满 width*height*2 字节"，也就是沿用 C 版的整屏读法
    #[test]
    fn single_solid_layer_fills_screen() {
        let mut vram = FakeVram::new();
        vram.put_solid(L0, N, RED);
        let mut lcd = new_lcd();
        arm_addr(&mut lcd, &[Some(L0)]);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb), "该更新时返回了 false");
        assert_eq!(fb, vec![0x00FF_0000; N]);
        assert_eq!(vram.reads, vec![(L0, N * BPP)], "读取的基址/长度应该就是这层的大小");
        assert!(!lcd.dirty, "合成过一次就该把 dirty 落回 false");
    }

    /// 显存里的 u16 是小端（低字节在前）。按大端读的话这两个像素的结果会互换，
    /// 整屏偏色且色键位置错乱
    #[test]
    fn pixels_are_little_endian() {
        let mut vram = FakeVram::new();
        let base = (L0 - RAM) as usize;
        // 像素 0：字节 00 F8 → 值 0xF800 → 纯红
        vram.bytes[base] = 0x00;
        vram.bytes[base + 1] = 0xF8;
        // 像素 1：字节 1F 00 → 值 0x001F → 是色键，必须透过去（保持哨兵）
        vram.bytes[base + 2] = 0x1F;
        vram.bytes[base + 3] = 0x00;
        let mut lcd = new_lcd();
        arm_addr(&mut lcd, &[Some(L0)]);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert_eq!(fb[0], 0x00FF_0000, "小端解释错了：像素 0 不是纯红");
        assert_eq!(fb[1], SENTINEL, "0x1F 00 按小端是色键 0x001F，不该被画");
    }

    // ---------- 色键 ----------

    /// 一整层都是色键时，下层必须完好无损——色键是"跳过"，不是"画成透明黑"
    #[test]
    fn key_color_leaves_lower_layer_intact() {
        let mut vram = FakeVram::new();
        vram.put_screen(L0, RED, &[]);
        vram.put_screen(L1, BLUE, &[]); // 整层都是默认色键
        let mut lcd = new_lcd();
        arm_addr(&mut lcd, &[Some(L0), Some(L1)]);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert_eq!(fb, vec![0x00FF_0000; N], "色键层把下层盖掉了");
    }

    /// LxWINKEY 编程过就以编程值为准：写 0x0000 之后黑色变透明，原来的 0x001F 反而要画出来
    #[test]
    fn programmed_winkey_replaces_default_key() {
        let mut vram = FakeVram::new();
        vram.put_screen(L0, RED, &[]);
        vram.put_screen(L1, 0x0000, &[(1, BLUE)]); // 整层黑，只有像素 1 是 0x001F

        // 没编程 WINKEY：黑色不透明（盖住红），0x001F 透明
        let mut plain = new_lcd();
        arm_addr(&mut plain, &[Some(L0), Some(L1)]);
        let mut fb = blank_fb();
        assert!(comp(&mut plain, &mut vram, &mut fb));
        assert_eq!(fb[0], 0x0000_0000, "默认色键是 0x001F，黑色本该照常显示");
        assert_eq!(fb[1], 0x00FF_0000);

        // 编程 WINKEY = 0x0000：黑色透明、0x001F 正常显示成纯蓝
        let mut keyed = new_lcd();
        arm_addr(&mut keyed, &[Some(L0), Some(L1)]);
        keyed.write_reg(reg(1, OFF_WINKEY), 0x0000);
        let mut fb = blank_fb();
        assert!(comp(&mut keyed, &mut vram, &mut fb));
        assert_eq!(fb[0], 0x00FF_0000, "编程色键没生效：黑色没被透掉");
        assert_eq!(fb[1], 0x0000_00FF, "0x001F 不再是色键，该画成纯蓝");
    }

    // ---------- 叠加顺序 ----------

    /// 索引大的在上层，而且是逐像素：同一像素 L2 盖 L1、L1 盖 L0；
    /// 各层的色键区域要各自露出更下面的层
    #[test]
    fn higher_index_layer_wins_per_pixel() {
        let mut vram = FakeVram::new();
        vram.put_screen(L0, RED, &[]);
        // L1 只画像素 5（绿）和 6（蓝 30/31），其余色键
        vram.put_screen(L1, BLUE, &[(5, GREEN), (6, 0x001E)]);
        // L2 只画像素 5（白）
        vram.put_screen(L2, BLUE, &[(5, WHITE)]);
        let mut lcd = new_lcd();
        arm_addr(&mut lcd, &[Some(L0), Some(L1), Some(L2)]);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert_eq!(fb[5], 0x00FF_FFFF, "L2 的白没盖住 L1 的绿");
        assert_eq!(fb[6], 0x0000_00F7, "L1 该盖住 L0 的红");
        assert_eq!(fb[0], 0x00FF_0000, "L1/L2 的色键像素不该盖住 L0");
        assert_eq!(fb[10], 0x00FF_0000);
    }

    /// 反过来也得成立：底层的普通像素不能被上层的色键"擦掉"
    #[test]
    fn upper_layer_key_pixel_does_not_erase_lower() {
        let mut vram = FakeVram::new();
        vram.put_screen(L0, GREEN, &[]);
        vram.put_screen(L1, BLUE, &[(0, WHITE)]);
        let mut lcd = new_lcd();
        arm_addr(&mut lcd, &[Some(L0), Some(L1)]);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert_eq!(fb[0], 0x00FF_FFFF, "L1 的非色键像素该盖住 L0");
        assert_eq!(fb[1..], vec![0x0000_FF00; N - 1][..], "L1 的色键像素把 L0 擦掉了");
    }

    /// 地址为 0（固件没配这层）的图层跳过，而且一个字节都不去读
    #[test]
    fn layer_with_zero_address_is_skipped_without_reading() {
        let mut vram = FakeVram::new();
        vram.put_solid(L1, N, GREEN); // 内容在内存里，但这层没被配地址
        vram.put_solid(L2, N, 0x001E);
        let mut lcd = new_lcd();
        arm_addr(&mut lcd, &[None, None, Some(L2)]);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert_eq!(fb, vec![rgb565_to_argb(0x001E); N]);
        assert_eq!(vram.reads, vec![(L2, N * BPP)], "只该读配了地址的那层");
    }

    // ---------- dirty 语义 ----------

    /// 只有 FRAME_TRANSFER 写 0 才需要重绘：配地址、写别的值都不该触发；
    /// 而且"已经挂着 dirty"时后来的非 0 写不能取消这次重绘
    #[test]
    fn dirty_is_set_only_by_frame_transfer_zero() {
        let mut lcd = new_lcd();
        assert!(!lcd.dirty);
        lcd.write_reg(reg(0, OFF_WINADD), L0);
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 1);
        assert!(!lcd.dirty, "帧传输写非 0 不该重绘");
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 0);
        assert!(lcd.dirty, "帧传输写 0 该重绘");
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 1);
        assert!(lcd.dirty, "已挂起的重绘不该被后来的非 0 写取消");
        lcd.write_reg(reg(1, OFF_WINADD), L1);
        assert!(lcd.dirty, "配图层地址不该顺手清掉 dirty");
    }

    /// `LCD_CON` 的 bit0（`LCD_ON`）读 0 —— 这一半是被复位握收钉死的：`0x0839_AF00` 的
    /// `while ((u16)[0x90000000] & 1) ;` 要求它为 0 才走得出（实测恒答 1 会吃掉之后全部深度）。
    ///
    /// **但别据此以为"两个读者都要 0"**：GDI 提交门 `0x0813_1E3A` 要的是相反的 —— bit0 为 1
    /// 就直接提交，为 0 才退而去问 `[0xF01D2A5C] ∈ {0x17,0x18}`。所以答 0 只满足了握收，
    /// 图层提交必须由状态字节自己推进；旧笔记把这条写成"两个读者都要 0"，我据此回退过一次
    /// 状态机尝试。也不要拿它当"正在传帧"—— 帧传送是另一个寄存器（`+0xC`）。
    #[test]
    fn lcd_on_bit_reads_zero_for_the_reset_handshake() {
        let mut lcd = new_lcd();
        assert_eq!(
            lcd.read(lcd_reg::DISP_STAT),
            Some(0),
            "复位握收 `0x0839_AF00` 要求 bit0 为 0"
        );
        assert_ne!(lcd.read(lcd_reg::DISP_STAT), Some(lcd_reg::DISP_STAT_RUNNING));
        // 提交一帧也不该把这个位翻成 1
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 1);
        assert_eq!(lcd.read(lcd_reg::DISP_STAT), Some(0), "帧传送寄存器不参与 LCD_ON 的作答");
    }

    /// 合成一次之后 dirty 落回 false；第二次调用既不返回 true，也不读显存、也不动 fb
    #[test]
    fn composite_consumes_dirty() {
        let mut vram = FakeVram::new();
        vram.put_solid(L0, N, RED);
        let mut lcd = new_lcd();
        arm_addr(&mut lcd, &[Some(L0)]);
        let mut fb = blank_fb();
        assert!(comp(&mut lcd, &mut vram, &mut fb));

        vram.put_solid(L0, N, GREEN); // 显存改了，但没有新的帧传输
        vram.reads.clear();
        fb.iter_mut().for_each(|p| *p = SENTINEL);
        assert!(!comp(&mut lcd, &mut vram, &mut fb), "不 dirty 时不该说更新了");
        assert!(vram.reads.is_empty(), "不 dirty 时不该读显存");
        assert_eq!(fb, vec![SENTINEL; N], "不 dirty 时不该动帧缓冲");
    }

    /// 帧缓冲装不下一屏时返回 false，但 **dirty 必须保留**，否则这一帧永久丢失
    #[test]
    fn short_framebuffer_does_not_drop_the_frame() {
        let mut vram = FakeVram::new();
        vram.put_solid(L0, N, RED);
        let mut lcd = new_lcd();
        arm_addr(&mut lcd, &[Some(L0)]);

        let mut small = blank_fb();
        small.pop();
        assert!(!comp(&mut lcd, &mut vram, &mut small), "装不下就不该说更新了");
        assert!(vram.reads.is_empty(), "装不下时不该去读显存");
        assert!(lcd.dirty, "装不下却清了 dirty，等于丢帧");

        let mut fb = blank_fb();
        assert!(comp(&mut lcd, &mut vram, &mut fb), "换了够大的 fb 就该补上这一帧");
        assert_eq!(fb[0], 0x00FF_0000);
        assert!(!lcd.dirty);
    }

    /// 一层都没配地址时也算把这次帧传输消费掉了，但不能碰显存
    #[test]
    fn frame_transfer_without_any_layer_consumes_dirty() {
        let mut lcd = new_lcd();
        arm_addr(&mut lcd, &[None]);
        let mut vram = FakeVram::new();
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert!(vram.reads.is_empty());
        assert_eq!(fb, vec![SENTINEL; N]);
        assert!(!lcd.dirty);
    }

    // ---------- 寄存器解码 ----------

    /// 模块头说的"块基址 0xB0、步长 0x30、WINADD 在 +0x0C"必须与 memmap 里 vm 实际匹配
    /// 的那四个常量一致。这条一旦不一致，图层顺序会静默错位
    #[test]
    fn layer_block_layout_matches_memmap_constants() {
        let adds = [
            (0usize, lcd_reg::L0_ADDRESS),
            (1, lcd_reg::L1_ADDRESS),
            (2, lcd_reg::L2_ADDRESS),
            (3, lcd_reg::L3_ADDRESS),
        ];
        for (i, constant) in adds {
            assert_eq!(reg(i, OFF_WINADD), constant, "层 {i} 的 WINADD 与 memmap 不一致");
        }
        assert_eq!(reg(1, OFF_WINADD) - reg(0, OFF_WINADD), LAYER_STRIDE);
        assert_eq!(reg(2, OFF_WINADD) - reg(1, OFF_WINADD), LAYER_STRIDE);
        assert_eq!(reg(3, OFF_WINADD) - reg(2, OFF_WINADD), LAYER_STRIDE);
        assert_eq!(OFF_WINADD, 0x0C, "WINADD 必须在层块内 +0x0C，否则整块划分都错位");
    }

    /// 四个地址寄存器各就各位，别串层
    #[test]
    fn each_address_register_targets_its_own_layer() {
        let mut lcd = new_lcd();
        lcd.write_reg(lcd_reg::L0_ADDRESS, L0);
        lcd.write_reg(lcd_reg::L1_ADDRESS, L1);
        lcd.write_reg(lcd_reg::L2_ADDRESS, L2);
        lcd.write_reg(lcd_reg::L3_ADDRESS, L3);
        assert_eq!(lcd.layers.map(|l| l.addr), [L0, L1, L2, L3], "图层地址串位了");
    }

    /// 每层的 KEY/OFS/SIZE/PITCH 也要落在正确的层上；块外的地址不许被当成图层寄存器
    #[test]
    fn geometry_registers_land_on_right_layer() {
        let mut lcd = new_lcd();
        lcd.write_reg(reg(2, OFF_WINADD), L2);
        lcd.write_reg(reg(2, OFF_WINKEY), 0x1234);
        lcd.write_reg(reg(2, OFF_WINOFS), ofs(3, 7)); // 请求位置 X=3, Y=7（寄存器里存的是 +1024）
        lcd.write_reg(reg(2, OFF_WINSIZE), 0x000A_0006); // ROW=10, COLUMN=6
        lcd.write_reg(reg(2, OFF_WINPITCH), 0x14);
        let l2 = lcd.layers[2];
        assert_eq!((l2.x, l2.y), (3, 7), "X-OFFSET 在低半字");
        assert_eq!((l2.width, l2.height), (6, 10), "COLUMN 在低半字");
        assert_eq!(l2.key, Some(0x1234));
        assert_eq!(l2.pitch, 0x14);
        assert_eq!(l2.addr, L2);
        assert_eq!(lcd.layers[0], Layer::default(), "写层 2 不该动层 0");
        assert_eq!(lcd.layers[3], Layer::default(), "写层 2 不该动层 3");

        let mut after = new_lcd();
        after.write_reg(LAYER_BASE - 4, L1); // 图层块区之前
        // 最后一块之后（0x9000_0170 是 LCD_DITHER_CON，另一个功能块）
        after.write_reg(LAYER_BASE + 4 * LAYER_STRIDE, L1);
        assert_eq!(after.layers, [Layer::default(); 4], "块外地址被当成图层寄存器了");
    }

    /// vm 传的是"实际访问地址"，字节/半字写在某个寄存器中间时低位非 0；
    /// 不做对齐的话这类写会被整个丢掉
    #[test]
    fn unaligned_access_address_is_masked() {
        let mut lcd = new_lcd();
        lcd.write_reg(reg(1, OFF_WINADD) + 1, L1);
        assert_eq!(lcd.layers[1].addr, L1, "子字写合并后的整字没被认下来");
        lcd.write_reg(lcd_reg::FRAME_TRANSFER + 3, 0);
        assert!(lcd.dirty, "帧寄存器的非对齐写被丢了");
    }

    // ---------- 每层几何 ----------

    /// 上层只画自己那一块：位置看 WINOFS、大小看 WINSIZE、显存行跨距看 WINPITCH。
    /// 窗口以外和跨距填充部分的像素都不许漏到屏幕上
    #[test]
    fn layer_geometry_offset_size_pitch_are_honored() {
        let mut vram = FakeVram::new();
        vram.put_screen(L0, RED, &[]);
        // 2x2 的窗口贴到 (x=1, y=1)，即屏幕像素 5、6、9、10；显存每行 4 像素，后两列是填充
        vram.put_rows(
            L1,
            8,
            &[&[GREEN, WHITE, JUNK, JUNK][..], &[0x001E, BLUE, JUNK, JUNK][..]],
        );
        let mut lcd = new_lcd();
        lcd.write_reg(reg(0, OFF_WINADD), L0);
        lcd.write_reg(reg(1, OFF_WINADD), L1);
        lcd.write_reg(reg(1, OFF_WINOFS), ofs(1, 1));
        lcd.write_reg(reg(1, OFF_WINSIZE), (2 << 16) | 2);
        lcd.write_reg(reg(1, OFF_WINPITCH), 8);
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 0);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert_eq!(fb[5], 0x0000_FF00, "窗口左上角没贴到 (x=1, y=1)");
        assert_eq!(fb[6], 0x00FF_FFFF);
        assert_eq!(fb[9], rgb565_to_argb(0x001E), "跨距错一行就会读到填充");
        assert_eq!(fb[10], 0x00FF_0000, "窗口里的色键该露出下层");
        for i in [0usize, 1, 2, 3, 4, 7, 8, 11] {
            assert_eq!(fb[i], 0x00FF_0000, "窗口外的像素 {i} 被上层盖了");
        }
        // 最后一行只读到用得上的 2 个像素：(2-1)*8 + 2*2 = 12
        assert_eq!(vram.reads, vec![(L0, N * BPP), (L1, 12)]);
    }

    /// WINPITCH 没编程时跨距按**本层宽度**推，不是按屏幕宽度
    #[test]
    fn default_pitch_comes_from_layer_width() {
        let mut vram = FakeVram::new();
        vram.put_screen(L0, RED, &[]);
        // 2x2 窗口、紧密排列（跨距 4 字节）
        vram.put_rows(L1, 4, &[&[GREEN, WHITE][..], &[0x001E, JUNK][..]]);
        let mut lcd = new_lcd();
        lcd.write_reg(reg(0, OFF_WINADD), L0);
        lcd.write_reg(reg(1, OFF_WINADD), L1);
        lcd.write_reg(reg(1, OFF_WINSIZE), (2 << 16) | 2);
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 0);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert_eq!(fb[0], 0x0000_FF00);
        assert_eq!(fb[1], 0x00FF_FFFF);
        assert_eq!(fb[4], rgb565_to_argb(0x001E), "跨距按屏幕宽推的话会读到下一行中间");
        assert_eq!(fb[5], rgb565_to_argb(JUNK));
        assert_eq!(fb[8], 0x00FF_0000, "窗口只有 2 行，第 3 行不该画");
        assert_eq!(vram.reads[1], (L1, 8), "need 应该是 (2-1)*4 + 2*2");
    }

    /// 窗口比屏幕大时要被裁掉，不许回卷到下一行开头
    #[test]
    fn oversized_window_is_clipped() {
        let mut vram = FakeVram::new();
        vram.put_screen(L0, RED, &[]);
        // 10x10 的窗口，屏幕只有 4x3；显存跨距 = 10 像素
        let rows: Vec<Vec<u16>> =
            (0..10).map(|r| (0..10).map(|c| color_of(r, c)).collect()).collect();
        let refs: Vec<&[u16]> = rows.iter().map(|r| &r[..]).collect();
        vram.put_rows(L1, 10 * BPP, &refs);
        let mut lcd = new_lcd();
        lcd.write_reg(reg(0, OFF_WINADD), L0);
        lcd.write_reg(reg(1, OFF_WINADD), L1);
        lcd.write_reg(reg(1, OFF_WINSIZE), (10 << 16) | 10);
        lcd.write_reg(reg(1, OFF_WINPITCH), 10 * BPP as u32);
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 0);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        for y in 0..H as usize {
            for x in 0..W as usize {
                assert_eq!(
                    fb[y * W as usize + x],
                    rgb565_to_argb(color_of(y, x)),
                    "屏幕 ({x}, {y}) 处来的像素不对"
                );
            }
        }
        // 只读用得上的部分：(3-1)*20 + 4*2 = 48；按整层读就是 200 字节
        assert_eq!(vram.reads[1], (L1, 48));
    }

    /// 整层落在屏幕外：不画，也一个字节都不读
    #[test]
    fn fully_offscreen_layer_is_not_read() {
        let mut vram = FakeVram::new();
        vram.put_solid(L1, N, GREEN);
        let mut lcd = new_lcd();
        lcd.write_reg(reg(1, OFF_WINADD), L1);
        lcd.write_reg(reg(1, OFF_WINOFS), ofs(W as i32, 0)); // X = 屏宽，整层在屏幕右边外
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 0);
        let mut fb = blank_fb();

        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert!(vram.reads.is_empty(), "屏幕外的层不该去读显存");
        assert_eq!(fb, vec![SENTINEL; N]);
    }

    /// 回归：`LxWINOFS` 存的是"请求坐标 + 1024"，**不是**裸像素坐标。
    /// 本固件把 L0 放在请求位置 (0,0)，于是寄存器实测值是 `0x0400_0400`。
    /// 当成无符号坐标用 ⇒ `x=1024 >= 屏宽` ⇒ 整层被跳过 ⇒ **画面必然全黑**；
    /// 而 C 版参考根本不读 WINOFS，所以它总能贴上去 —— 两版行为不等价的那一处。
    #[test]
    fn winofs_is_biased_by_1024_so_the_firmware_zero_position_still_composites() {
        let mut vram = FakeVram::new();
        vram.put_solid(L0, N, RED);
        let mut lcd = new_lcd();
        lcd.write_reg(reg(0, OFF_WINADD), L0);
        lcd.write_reg(reg(0, OFF_WINOFS), 0x0400_0400); // 固件实测写进寄存器的原值
        lcd.write_reg(lcd_reg::FRAME_TRANSFER, 0);

        assert_eq!((lcd.layers[0].x, lcd.layers[0].y), (0, 0), "零偏没被减掉");
        let mut fb = blank_fb();
        assert!(comp(&mut lcd, &mut vram, &mut fb));
        assert_eq!(fb[0], 0x00FF_0000, "请求位置 (0,0) 的层被整层跳过了");
        assert_eq!(fb[N - 1], 0x00FF_0000, "只贴了半个屏幕");
    }
}
