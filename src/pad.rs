//! 屏下虚拟按键面板：布局、绘制、鼠标命中测试。
//!
//! 窗口不可缩放，所以这里的坐标就是物理像素，不用再做一层缩放飞地，也就不用
//! 处理 DPI 变化时按键区域错位的问题。
//!
//! 标签只用位图字体画得出数字，所以方向键一律画成三角形，其余能凑成词的用
//! `SL/SR/OK/SEND/END`（见 `FONT`），避免为几个汉字再引入真正的字体渲染。

use crate::devices::keypad::{self, code};

const BTN_W: u32 = 72;
const BTN_H: u32 = 26;
const GAP: u32 = 8;
const ROW_H: u32 = BTN_H + GAP;
const DIGIT_ROW_H: u32 = BTN_H + 6;
/// 面板顶部留白（LCD 与第一排按键之间）
const TOP: u32 = 10;

const BG: u32 = 0xff1b_1e23;
const BTN: u32 = 0xff2f_3540;
const BTN_HELD: u32 = 0xff5b_6a84;
const EDGE: u32 = 0xff44_4c5a;
const INK: u32 = 0xffd8_dee9;

/// 面板自身的高度（不含 LCD 那一段）
pub const fn height() -> u32 {
    TOP + 3 * ROW_H + 4 * DIGIT_ROW_H + 4
}

#[derive(Clone, Copy, Debug)]
pub enum Face {
    Text(&'static str),
    Up,
    Down,
    Left,
    Right,
}

#[derive(Clone, Copy, Debug)]
pub struct Button {
    x: u32,
    y: u32,
    key: u8,
    face: Face,
}

pub struct Pad {
    buttons: Vec<Button>,
    origin_y: u32,
}

impl Pad {
    /// `screen_w`/`screen_h` 是固件屏幕尺寸，面板接在它正下方
    pub fn new(screen_w: u32, screen_h: u32) -> Self {
        let top = screen_h + TOP;
        let left = (screen_w - 3 * BTN_W - 2 * GAP) / 2;
        let col = |i: u32| left + i * (BTN_W + GAP);
        let row = |i: u32| top + i * ROW_H;
        let digit_row = |i: u32| top + 3 * ROW_H + i * DIGIT_ROW_H;
        let mut buttons = vec![
            btn(col(0), row(0), code::SOFT_LEFT, Face::Text("SL")),
            btn(col(1), row(0), code::END, Face::Text("END")),
            btn(col(2), row(0), code::SOFT_RIGHT, Face::Text("SR")),
            btn(col(0), row(1), code::LEFT, Face::Left),
            btn(col(1), row(1), code::UP, Face::Up),
            btn(col(2), row(1), code::RIGHT, Face::Right),
            btn(col(0), row(2), code::SEND, Face::Text("SEND")),
            btn(col(1), row(2), code::DOWN, Face::Down),
            btn(col(2), row(2), code::OK, Face::Text("OK")),
        ];
        for (i, label) in ['1', '2', '3', '4', '5', '6', '7', '8', '9', '*', '0', '#']
            .iter()
            .enumerate()
        {
            let (r, c) = ((i / 3) as u32, (i % 3) as u32);
            let label = digit_label(*label);
            let key = keypad::key_of(label).unwrap_or(code::STAR);
            buttons.push(btn(col(c), digit_row(r), key, Face::Text(label)));
        }
        Self { buttons, origin_y: top }
    }

    /// 命中测试：窗口坐标 → 键码
    pub fn hit(&self, x: u32, y: u32) -> Option<u8> {
        if y < self.origin_y {
            return None;
        }
        self.buttons
            .iter()
            .find(|b| (b.x..b.x + BTN_W).contains(&x) && (b.y..b.y + BTN_H).contains(&y))
            .map(|b| b.key)
    }

    /// 把面板画进帧缓冲（`stride` 是整窗宽度）
    pub fn draw(&self, buf: &mut [u32], stride: u32, held: Option<u8>) {
        for b in &self.buttons {
            let fill = if held == Some(b.key) { BTN_HELD } else { BTN };
            fill_rect(buf, stride, b.x, b.y, BTN_W, BTN_H, fill);
            outline_rect(buf, stride, b.x, b.y, BTN_W, BTN_H, EDGE);
            match b.face {
                Face::Text(t) => {
                    let scale = 2;
                    let w = text_width(t, scale);
                    draw_text(buf, stride, b.x + (BTN_W - w) / 2, b.y + (BTN_H - 7 * scale) / 2, t, scale, INK);
                }
                Face::Up => triangle(buf, stride, b.x, b.y, INK, 0),
                Face::Down => triangle(buf, stride, b.x, b.y, INK, 1),
                Face::Left => triangle(buf, stride, b.x, b.y, INK, 2),
                Face::Right => triangle(buf, stride, b.x, b.y, INK, 3),
            }
        }
    }
}

fn btn(x: u32, y: u32, key: u8, face: Face) -> Button {
    Button { x, y, key, face }
}

/// 单个字符要变成 `'static` 才能进 `Face::Text`；字符集固定是 12 个，每个只泄漏一次
fn digit_label(c: char) -> &'static str {
    let mut s = String::new();
    s.push(c);
    Box::leak(s.into_boxed_str())
}

/// 面板底色（LCD 区域之外的那部分窗口）
pub const BACKGROUND: u32 = BG;

fn text_width(s: &str, scale: u32) -> u32 {
    let n = s.chars().count() as u32;
    if n == 0 {
        0
    } else {
        n * (GLYPH_W + 1) * scale - scale
    }
}

const GLYPH_W: u32 = 5;
const GLYPH_H: u32 = 7;

fn draw_text(buf: &mut [u32], stride: u32, x: u32, y: u32, s: &str, scale: u32, color: u32) {
    let mut cx = x;
    for ch in s.chars() {
        for (r, bits) in glyph(ch).iter().enumerate() {
            for c in 0..GLYPH_W {
                if bits & (1 << (GLYPH_W - 1 - c)) == 0 {
                    continue;
                }
                fill_rect(
                    buf,
                    stride,
                    cx + c * scale,
                    y + r as u32 * scale,
                    scale,
                    scale,
                    color,
                );
            }
        }
        cx += (GLYPH_W + 1) * scale;
    }
}

/// 方向三角形：0=上 1=下 2=左 3=右，画在按钮中央 18x14 的框里。
/// 上下按"行宽从 0 放大到 w"铺，左右按"列高从 0 放大到 h"铺，两个方向的长宽不能混用。
fn triangle(buf: &mut [u32], stride: u32, bx: u32, by: u32, color: u32, dir: u8) {
    let (w, h) = (18u32, 14u32);
    let ox = bx + (BTN_W - w) / 2;
    let oy = by + (BTN_H - h) / 2;
    for i in 0..h {
        let span = (i + 1) * w / h;
        let x = ox + (w - span) / 2;
        match dir {
            0 => fill_rect(buf, stride, x, oy + i, span, 1, color),
            1 => fill_rect(buf, stride, x, oy + h - 1 - i, span, 1, color),
            _ => {}
        }
    }
    for j in 0..w {
        // ◀ 的尖在左：越往右越高；▶ 相反
        let span = if dir == 2 { (j + 1) * h / w } else { (w - j) * h / w };
        match dir {
            2 | 3 => fill_rect(buf, stride, ox + j, oy + (h - span) / 2, 1, span, color),
            _ => {}
        }
    }
}

fn fill_rect(buf: &mut [u32], stride: u32, x: u32, y: u32, w: u32, h: u32, color: u32) {
    for row in buf.chunks_mut(stride as usize).skip(y as usize).take(h as usize) {
        if let Some(slice) = row.get_mut(x as usize..(x + w).min(row.len() as u32) as usize) {
            slice.iter_mut().for_each(|p| *p = color);
        }
    }
}

fn outline_rect(buf: &mut [u32], stride: u32, x: u32, y: u32, w: u32, h: u32, color: u32) {
    fill_rect(buf, stride, x, y, w, 1, color);
    fill_rect(buf, stride, x, y + h - 1, w, 1, color);
    fill_rect(buf, stride, x, y, 1, h, color);
    fill_rect(buf, stride, x + w - 1, y, 1, h, color);
}

/// 5x7 点阵，bit4 在最左。只需要数字、`*#` 和拼出 SL/SR/OK/SEND/END 的字母
fn glyph(ch: char) -> [u8; GLYPH_H as usize] {
    match ch {
        '0' => [0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e],
        '1' => [0x04, 0x06, 0x04, 0x04, 0x04, 0x04, 0x0e],
        '2' => [0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f],
        '3' => [0x0e, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0e],
        '4' => [0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02],
        '5' => [0x1f, 0x10, 0x1c, 0x01, 0x01, 0x11, 0x0e],
        '6' => [0x06, 0x08, 0x10, 0x1c, 0x11, 0x11, 0x0e],
        '7' => [0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
        '8' => [0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e],
        '9' => [0x0e, 0x11, 0x11, 0x1e, 0x01, 0x02, 0x0c],
        '*' => [0x00, 0x04, 0x15, 0x0e, 0x15, 0x04, 0x00],
        '#' => [0x0a, 0x0a, 0x1f, 0x0a, 0x1f, 0x0a, 0x0a],
        'S' => [0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f],
        'R' => [0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11],
        'O' => [0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e],
        'K' => [0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11],
        'E' => [0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f],
        'N' => [0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11],
        'D' => [0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e],
        _ => [0x00; GLYPH_H as usize],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn buttons_fit_and_hit_themselves() {
        let (w, h) = (240u32, 320u32);
        let pad = Pad::new(w, h);
        let win_h = h + height();
        assert!(!pad.buttons.is_empty());
        for b in &pad.buttons {
            assert!(b.x + BTN_W <= w, "按键 {b:?} 超出窗口宽度");
            assert!(b.y + BTN_H <= win_h, "按键 {b:?} 超出窗口高度");
            assert_eq!(pad.hit(b.x + BTN_W / 2, b.y + BTN_H / 2), Some(b.key));
        }
        // 屏幕区域里没有键
        assert_eq!(pad.hit(120, 100), None);
    }

    /// 整块面板都画一遍：四种方向的三角形和所有标签，越界/下溢只有在这种全量绘制里才会暴露
    #[test]
    fn draw_whole_pad() {
        let (w, h) = (240u32, 320u32);
        let pad = Pad::new(w, h);
        let mut buf = vec![BACKGROUND; (w * (h + height())) as usize];
        for dir in 0..4u8 {
            let held = pad.buttons.iter().map(|b| b.key).nth(dir as usize);
            pad.draw(&mut buf, w, held);
        }
        let inked = buf.iter().filter(|&&p| p == INK).count();
        assert!(inked > 500, "面板几乎没画出东西: {inked}");
    }

    /// 方向三角形的朝向不能画反（截图里发现过 ◀/▶ 画成同一个方向的问题）
    #[test]
    fn arrows_point_the_right_way() {
        let ink = |dir: u8, col: u32| -> usize {
            let mut buf = vec![0u32; (BTN_W * BTN_H) as usize];
            triangle(&mut buf, BTN_W, 0, 0, INK, dir);
            (0..BTN_H).filter(|row| buf[(row * BTN_W + col) as usize] == INK).count()
        };
        // 三角形居中在 (72-18)/2 = 27 起的 18 列里
        assert!(ink(2, 27) < ink(2, 44), "◀ 应该是左边尖、右边高");
        assert!(ink(3, 27) > ink(3, 44), "▶ 应该是左边高、右边尖");
    }

    /// 点阵字体的行宽必须正好 5 列，否则标签会挤在一起；顺便把渲染结果打出来目视检查
    #[test]
    fn glyph_rows_are_five_columns() {
        for ch in "0123456789*#SLROKENDWV".chars() {
            for bits in glyph(ch) {
                assert_eq!(bits & !0x1f, 0, "{ch} 的点阵超出一列 5 位: {bits:#08b}");
            }
        }
        let mut buf = vec![crate::pad::BACKGROUND; (BTN_W * BTN_H) as usize];
        fill_rect(&mut buf, BTN_W, 0, 0, BTN_W, BTN_H, BTN);
        draw_text(&mut buf, BTN_W, 6, 6, "SEND", 2, INK);
        let mut art = String::new();
        for row in buf.chunks(BTN_W as usize) {
            art.push_str(&row.iter().map(|&p| if p == INK { '#' } else { '.' }).collect::<String>());
            art.push('\n');
        }
        println!("{art}");
        assert!(art.contains('#'), "标签没画出来");
    }
}
