//! MT6252 键盘矩阵。
//!
//! 注意一个与 C 版相反的所有权方向：C 版把寄存器写进"假内存"再让固件读回来，
//! 这里寄存器状态由设备自己持有。因为寄存器区间是 MMIO 后端，设备若再往那些地址
//! 写一次就会递归触发自己的写回调。
//!
//! 寄存器表取自数据手册 4.3.2（页 155~157）：
//! ```text
//! +0x00 KP_STA      RO  bit0=STA，1=有键按住。不是读清除，只由按键检测状态机改
//! +0x04 KP_MEM1  } 每层 16 位、共 5 层 80 个矩阵位；按下=0、松开=1，复位全 1
//! +0x14 KP_MEM5  }
//! +0x18 KP_DEBOUNCE  连续扫描间隔，由固件写
//! ```
//! 基址 `0x8107_0000` 的依据是 ROM 里确实有 3 处字面量指向它（文件偏移
//! `0x2144 / 0x37ac / 0x1f940`），其中 `0x0800_3768` 那个小助手按 `r1∈{0,1,2,3}` 选层，
//! `r1==4` 直接 `assert(0xC0C)` ⇒ **固件这一层只支持前 4 组 64 个矩阵位**，
//! 画像 72 项矩阵里索引 64..71 那 8 项固件读不到。`KP_MEM5` 仍然要答全 1，
//! 否则一旦有别的路径读到它就等于"8 颗键全按住"。

use crate::memmap::kpd;

#[derive(Debug, PartialEq)]
pub struct Keypad {
    /// 固件 `key_pad_comm_def->keypad[]`，索引即矩阵位编号，0xFE 表示空位
    pub matrix: Vec<u8>,
    /// 0x00 KP_STA：`bit0 = STA`，1 = 有键按下。**不是读清除**（手册 4.3.1 明说），
    /// 只能由按键检测状态机改；这里由 [`Keypad::press`] 代劳
    pub status: u32,
    /// 0x04/0x08/0x0c/0x10/0x14 = KP_MEM1..5，每层 16 位、共 80 个矩阵位，
    /// **按下为 0、松开为 1**（复位值全 1，见 [`Default`] 的说明）
    pub rows: [u16; 5],
    /// 0x18 KP_DEBOUNCE：连续按键扫描间隔
    pub period: u16,
}

/// 复位值不是全 0：**KP_STA = 0（没键按下）而 KP_MEM1..5 = 0xFFFF（每一位都"松开"）**。
/// 手册页 155/156 的 Reset 行就是 15 个 1。按 `derive(Default)` 让 `rows` 复位成 0，
/// 等于上电就报"64 个键全被按住"，固件的 `KP_STA` 判断目前恰好挡在前面才没暴露。
impl Default for Keypad {
    fn default() -> Self {
        Self { matrix: Vec::new(), status: 0, rows: [0xffff; 5], period: 0 }
    }
}

impl Keypad {
    /// 按键记账。`key` 是固件层的键码（0\~23 及功能键），不是矩阵下标。
    ///
    /// 不带 `Engine` 是故意的：与 [`crate::devices::lcd::Lcd::write_reg`] 同一套分工，
    /// 纯记账的部分要能在不起仿真进程的情况下单测。
    /// [`Keypad::press`] 的无 `Engine` 版本。返回值 = **寄存器状态有没有发生变化**：
    /// 手册 4.3.1 说 KEYPAD IRQ 是"状态变了且稳定"时才发，所以调用方只在 `true` 时投中断。
    ///
    /// `KP_STA` 跟着行线走：还有任何一位是 0（有键按住）就是 1，全松开就回 0。
    /// 以前它一旦被按下就**永远是 1**（开机那一次 power key 就够把它钉住），等于此后对所有
    /// 按键都谎报"还有一颗键按着没松"。
    pub fn press_key(&mut self, key: u8, down: bool) -> bool {
        let Some(index) = self.matrix.iter().position(|&k| k == key) else {
            println!("[keypad] 键码 {key:#x} 不在矩阵中，忽略");
            return false;
        };
        let before = (self.status, self.rows);
        let (group, bit) = (index / 16, index % 16);
        if down {
            self.rows[group] &= !(1 << bit) as u16; // 按下 = 该位拉低
        } else {
            self.rows[group] |= 1 << bit; // 松开 = 回到 1
        }
        self.status = if self.rows.iter().any(|&r| r != 0xffff) { 1 } else { 0 };
        (self.status, self.rows) != before
    }

    pub fn read(&self, addr: u32) -> Option<u32> {
        let off = addr - kpd::BASE;
        match off {
            0x00 => Some(self.status),
            0x04 | 0x08 | 0x0c | 0x10 | 0x14 => Some(self.rows[(off - 0x04) as usize / 4] as u32),
            0x18 => Some(self.period as u32),
            _ => None,
        }
    }

    pub fn write(&mut self, addr: u32, value: u32) {
        let off = addr - kpd::BASE;
        match off {
            0x00 => self.status = value,
            0x18 => self.period = value as u16,
            _ => {}
        }
    }
}

/// 固件键码，与画像 `keypad.matrix[]` 里存的值、C 版 `simulateKey` 编号一致。
/// 注意 `END` 就是画像里的 `boot.power_key`（0x17）：这颗机上"挂断"和"长按开机/关机"
/// 是同一颗键，短按挂断、长按开关机。
pub mod code {
    pub const STAR: u8 = 10;
    pub const HASH: u8 = 11;
    pub const UP: u8 = 14;
    pub const DOWN: u8 = 15;
    pub const LEFT: u8 = 16;
    pub const RIGHT: u8 = 17;
    pub const OK: u8 = 18;
    pub const SOFT_LEFT: u8 = 20;
    pub const SOFT_RIGHT: u8 = 21;
    pub const SEND: u8 = 22;
    pub const END: u8 = 23;
}

/// 宿主键盘（字符面）→ 固件键码。用字符而不是键位码，避免依赖 winit 的枚举，
/// 也避免主键盘/小键盘的数字键码不一致。
pub fn key_of(input: &str) -> Option<u8> {
    let c = input.chars().next()?.to_ascii_lowercase();
    let code = match c {
        '0'..='9' => c as u8 - b'0',
        'w' => code::UP,
        's' => code::DOWN,
        'a' => code::LEFT,
        'd' => code::RIGHT,
        'f' => code::OK,
        'q' => code::SOFT_LEFT,
        'e' => code::SOFT_RIGHT,
        'z' => code::SEND,
        'c' => code::END,
        'n' => code::STAR,
        'm' => code::HASH,
        _ => return None,
    };
    Some(code)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 手搓矩阵，不抄画像：索引 0 = 键码 0x12、索引 40（第 3 组第 8 位）= 键码 5、
    /// 索引 71（第 5 组第 7 位，**只有 KP_MEM5 才装得下**）= 键码 0x17
    fn matrix() -> Vec<u8> {
        let mut m = vec![0xfe; 72];
        m[0] = 0x12;
        m[40] = 5;
        m[71] = 0x17;
        m
    }

    fn kp() -> Keypad {
        Keypad { matrix: matrix(), ..Keypad::default() }
    }

    /// 复位态必须是"没键按下"：`KP_STA=0` 而 `KP_MEM1..5` 全 1。
    /// 这条以前是 `derive(Default)` 的全 0，等于上电报"80 个键全按住"
    #[test]
    fn reset_reads_no_key_pressed_and_all_lines_released() {
        let k = kp();
        assert_eq!(k.read(kpd::BASE), Some(0), "KP_STA 复位应为 0");
        for off in [0x04u32, 0x08, 0x0c, 0x10, 0x14] {
            assert_eq!(k.read(kpd::BASE + off), Some(0xffff), "KP_MEM{off:#x} 复位应全 1");
        }
        assert_eq!(k.read(kpd::BASE + 0x18), Some(0), "KP_DEBOUNCE 复位应为 0");
        // 本模型没建的偏移要回 None，让上层走后备存储，而不是假装有值
        assert_eq!(k.read(kpd::BASE + 0x1c), None);
    }

    /// 键码 → 矩阵位 → 哪一层哪一位。索引 40 落在**第 3 层**（`KP_MEM3` @ +0x0C）第 8 位，
    /// 且按下时只有那一位变 0，同层其它位仍是 1
    #[test]
    fn press_sets_one_bit_in_the_right_memory_group() {
        let mut k = kp();
        k.press_key(5, true);
        assert_eq!(k.read(kpd::BASE), Some(1), "KP_STA 该报有键按下");
        assert_eq!(k.rows[0], 0xffff);
        assert_eq!(k.rows[1], 0xffff);
        assert_eq!(k.rows[3], 0xffff);
        assert_eq!(k.rows[2], !(1 << 8) & 0xffff, "索引 40 = 40/16 组、40%16 位");
        assert_eq!(k.read(kpd::BASE + 0x0c), Some(k.rows[2] as u32));
    }

    /// 索引 ≥ 64 的键以前会被丢掉：`rows` 只有 4 层，循环到不了第 5 层，
    /// 而 `read(0x14)` 又回 None ⇒ 固件读 KP_MEM5 会拿到后备存储的 0 = "全按下"
    #[test]
    fn keys_in_the_fifth_group_reach_the_registers() {
        let mut k = kp();
        k.press_key(0x17, true);
        assert_eq!(k.rows[4], !(1 << 7) & 0xffff, "索引 71 = 第 4 组第 7 位");
        assert_eq!(k.read(kpd::BASE + 0x14), Some(k.rows[4] as u32), "KP_MEM5 必须可读");
    }

    /// 松开最后一颗键：行线回到全 1，`KP_STA` 也跟着落回 0（手册 4.3.1：STA 由检测状态机
    /// 按"还有没有键按住"来改，**不是**读清除、也不是按下一去就永久置着）。
    /// 以前是"按下置 1、松开不动"，等于开机那次 power key 之后一直谎报"有键按着"。
    #[test]
    fn releasing_the_last_key_drops_status() {
        let mut k = kp();
        assert!(k.press_key(0x12, true), "按下是状态变化");
        assert_eq!(k.rows[0] & 1, 0);
        assert_eq!(k.status, 1);
        assert!(k.press_key(0x12, false), "松开也是状态变化");
        assert_eq!(k.rows, [0xffff; 5], "松开后所有行线该回到 1");
        assert_eq!(k.status, 0, "没有键按住了，KP_STA 该落回 0");
    }

    /// 两颗键同时按住时，松开其中一颗 `KP_STA` 必须还是 1
    #[test]
    fn chording_keeps_status_until_the_last_release() {
        let mut k = kp();
        k.press_key(0x12, true); // 索引 0
        k.press_key(5, true); // 索引 40
        assert_eq!(k.status, 1);
        k.press_key(0x12, false);
        assert_eq!(k.status, 1, "还有一颗按着");
        assert_eq!(k.rows[2], !(1 << 8) & 0xffff, "另一颗键的行线不许被连带放开");
        k.press_key(5, false);
        assert_eq!(k.status, 0);
    }

    /// 重复的"按下同一颗键"不产生状态变化 ⇒ 调用方不该再投一次 KEYPAD IRQ
    /// （真机上 IRQ 只在状态**变化**时发）
    #[test]
    fn repeated_press_of_the_same_key_reports_no_change() {
        let mut k = kp();
        assert!(k.press_key(0x12, true));
        assert!(!k.press_key(0x12, true), "同一颗键按第二次不该报变化");
    }

    /// 矩阵里没有的键码不许把状态改脏
    #[test]
    fn unknown_key_code_leaves_state_untouched() {
        let mut k = kp();
        k.press_key(0x7b, true);
        assert_eq!(k, kp());
    }
}

