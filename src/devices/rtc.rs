//! RTC（0x810B_xxxx）：把时间刷进固件读取的七个字段。
//! 起点是常量、推进按固件的名义时间（片数）—— 见 [`Rtc::tick`] 和 [`FIXED_START`]。
use crate::memmap::rtc_reg;

#[repr(C)]
#[derive(Default)]
struct SystemTime {
    w_year: u16,
    w_month: u16,
    w_day_of_week: u16,
    w_day: u16,
    w_hour: u16,
    w_minute: u16,
    w_second: u16,
    w_milliseconds: u16,
}

unsafe extern "system" {
    fn GetLocalTime(time: *mut SystemTime);
}

/// 固件从 2000 年起算年份，这里做偏移修正
const EPOCH_YEAR: u32 = 2000;

/// 默认起点 (day, week, month, year−2000)：**2026-09-27，周日**（星期按 Windows
/// `SYSTEMTIME.wDayOfWeek` 的惯例，周日 = 0）。
///
/// 为什么不向宿主取：取一次也是把"这一轮跑在几点"塞进仿真的输入。两轮相隔 22 秒的
/// 实测里，pc / cpsr / 四个中断计数逐行全同，只有 RAM 摘要不同（`dump/g1.txt` vs
/// `g2.txt`）——差的正是被固件抄进内存结构体的那几秒。要做可复现基线，起点必须是常量。
/// 交互运行想看真实日期时设 `MT6252_RTC=host`。
const FIXED_START: (u32, u32, u32, u32) = (27, 0, 9, 26);

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Fields {
    pub sec: u32,
    pub min: u32,
    pub hour: u32,
    pub day: u32,
    pub week: u32,
    pub month: u32,
    /// 相对 2000 年
    pub year: u32,
}

#[derive(Debug, Default)]
pub struct Rtc {
    /// 2 = 秒计数器中断，1 = 闹钟中断
    pub irq_control: u32,
    pub irq_status: u32,
    pub fields: Fields,
    /// 起点当日已过秒数，默认 0（只在 `MT6252_RTC=host` 时取宿主值）
    start_tod: u32,
    /// 起点日历日 (day, week, month, year 相对 2000)，默认 [`FIXED_START`]
    start_date: (u32, u32, u32, u32),
    primed: bool,
}

/// 由"起点秒数 + 名义经过毫秒"算出时分秒。只回卷时刻、不进日历：走到待机屏只过了名义
/// 几十秒，而翻一天要真机挂满 24 小时，为此引入一套月份长度不值得。
fn clock_of(start_tod: u32, elapsed_ms: u64) -> (u32, u32, u32) {
    let secs = (u64::from(start_tod) + elapsed_ms / 1000) % 86_400;
    ((secs % 60) as u32, (secs / 60 % 60) as u32, (secs / 3600) as u32)
}

impl Rtc {
    /// 时间全部由**片数**推出来，宿主墙钟只在 `MT6252_RTC=host` 时参与起点。
    ///
    /// 理由：墙钟是仿真输入里唯一一个每轮都不一样的量。同一个 exe、同一套参数、探针全关
    /// 跑两遍，每片 500 ms 一次的 tick 各读到不同的秒数，固件抄进内存结构体的日期就跟着
    /// 不一样，轨迹指纹的 RAM 摘要因此分叉（控制流没分：pc / cpsr / 四个中断计数逐行全同）。
    /// 推进用"起点 + 名义经过时间"，500 片走 1 秒，和 `clock13` 同一把尺子。
    pub fn tick(&mut self, elapsed_ms: u64) {
        if !self.primed {
            if std::env::var("MT6252_RTC").is_ok_and(|v| v.trim() == "host") {
                let mut t = SystemTime::default();
                unsafe { GetLocalTime(&mut t) };
                self.start_tod =
                    u32::from(t.w_hour) * 3600 + u32::from(t.w_minute) * 60 + u32::from(t.w_second);
                self.start_date = (
                    u32::from(t.w_day),
                    u32::from(t.w_day_of_week),
                    u32::from(t.w_month),
                    u32::from(t.w_year).saturating_sub(EPOCH_YEAR),
                );
            } else {
                self.start_tod = 0;
                self.start_date = FIXED_START;
            }
            self.primed = true;
        }
        let (sec, min, hour) = clock_of(self.start_tod, elapsed_ms);
        let (day, week, month, year) = self.start_date;
        self.fields = Fields { sec, min, hour, day, week, month, year };
        // C 版在这里强制把控制字改成 2（秒计数器中断），保持同样的行为
        if self.irq_control != 2 {
            self.irq_control = 2;
            self.irq_status = 2;
            self.fields.sec = 0;
        }
    }

    /// 七个时间寄存器**故意不在这里**。C 版的 `Update_RTC_Time()` 是用 `uc_mem_write`
    /// 把宿主时间**推进内存**的（`main.c:283-295`，500 ms 一次），所以那些地址对固件来说
    /// 就是普通 RAM：固件自己写进去的值能回读，下一次推送再覆盖。
    /// 我们以前在 read 端现造返回值，等于把固件的写全部吞掉 —— 固件
    /// （`0x0803_5BF8` 等）把读回的时间和自己刚写的 7 字节逐字节比，必然对不上，
    /// 于是进 `0x0803_5C90` 那个"重植魔数 + 空转 500 次"的重试环，最多 0x989680 轮，
    /// 表现就是深度计数冻住、PC 在 `0x0803_5CB9` 空转。
    /// 现在 `None` ⇒ 落到 `Vm::reg_store`，由 [`Rtc::time_regs`] 在每个 RTC tick 写入。
    /// IRQ_STATUS 保持设备侧给出：C 版在控制字不是 2 时会**强行**往这里写 2
    /// （"秒计数器中断"），固件就是轮询它等第一次秒 tick。
    pub fn read(&self, addr: u32) -> Option<u32> {
        match addr {
            rtc_reg::IRQ_STATUS => Some(self.irq_status),
            _ => None,
        }
    }

    /// 供 `Vm` 推进 `reg_store` 的七个 (寄存器, 值) 对，语义等同于 C 版的 `uc_mem_write`。
    pub fn time_regs(&self) -> [(u32, u32); 7] {
        [
            (rtc_reg::SEC, self.fields.sec),
            (rtc_reg::MIN, self.fields.min),
            (rtc_reg::HOUR, self.fields.hour),
            (rtc_reg::DAY, self.fields.day),
            (rtc_reg::WEEK, self.fields.week),
            (rtc_reg::MONTH, self.fields.month),
            (rtc_reg::YEAR, self.fields.year),
        ]
    }

    pub fn write(&mut self, addr: u32, value: u32) {
        match addr {
            rtc_reg::IRQ_CONTROL => self.irq_control = value,
            rtc_reg::IRQ_STATUS => self.irq_status = value,
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clock_advances_only_with_nominal_time() {
        assert_eq!(clock_of(0, 0), (0, 0, 0));
        assert_eq!(clock_of(0, 999), (0, 0, 0), "不满一秒不动秒针");
        assert_eq!(clock_of(0, 1000), (1, 0, 0));
        assert_eq!(clock_of(59, 61_000), (0, 2, 0), "起点 59 秒 + 61 秒 = 02:00");
        assert_eq!(clock_of(0, 3_720_000), (0, 2, 1), "1 小时 2 分");
        assert_eq!(clock_of(23 * 3600 + 59 * 60 + 30, 60_000), (30, 0, 0), "跨午夜只回卷时刻");
    }

    #[test]
    fn tick_is_a_pure_function_of_elapsed_slices() {
        // 同一台机器上先后两次运行必须给出同样的字段 —— 这是可复现性基线的前提
        let mut a = Rtc::default();
        let mut b = Rtc::default();
        a.tick(4000);
        b.tick(4000);
        assert_eq!(a.fields, b.fields);
        assert_eq!(a.time_regs()[0].1, 0, "首次 tick 会按 C 版强制秒计数器，秒归零");
    }
}
