//! C1XX 13 MHz 系统定时器（0x8200_xxxx）。
//!
//! 比较匹配型中断，**而且比较值是硬件自动重装的**：全 ROM 只有一条指令写
//! `REAL_TIME_COMPARE`（`0x080019B0`，开机写 `0x8072`），再没有任何第二次写、也没有任何读。
//! 所以周期必须由硬件从 `0x238` 自己重装，不能等软件重挂 —— 之前"投一次就再也不投"
//! 就是按"等软件写新比较值"建模造成的。
//!
//! `0x82000200` / `0x204` 是**只写的 32 位数据**（hi16 / lo16，唯一写者 `0x08034FAC`，
//! 用于 L1SM 睡眠唤醒的时刻补偿），**不是中断状态寄存器**：全 ROM 对这两个地址零读取，
//! 而固件确实会往里写。之前把它当"读到即清的 INT_STATUS"，等于固件每装载一次补偿值
//! 就把我们的中断状态误清一次。ISR 判断中断源靠的是中断控制器的 `INT_STATUS`
//! （`0x810100D8`），不是 C1XX 的寄存器。
//!
//! `0x8200021C`（一次性延时的状态/ack）**故意不建模**：固件对它的读回落到 `reg_store`
//! 恰好等价于 C 参考版的行为（C 把整段映射成普通 RAM），而凭猜测置 bit4~bit9 会让固件
//! 的分流逻辑静默吞掉中断。

use crate::memmap::systimer;

#[derive(Debug, Default)]
pub struct C1xx {
    /// 重装周期 = 固件最后一次写进 `COMPARE` 的值；0 = 还没编程过，永不匹配
    period: u32,
    /// 已经服务到第几个周期。和 `counter / period` 一比就知道有没有跨过新的匹配沿。
    served: u32,
    /// 计数器换算基准，见 [`C1xx::arm`]
    epoch: u32,
    /// TDMA 帧号的换算基准，见 [`C1xx::rearm_tdma`]
    tdma_epoch: u32,
    /// TDMA 帧号本体，每读一次 +1，见 [`C1xx::next_frame`]
    frame: u32,
}

impl C1xx {
    /// `counter_raw` 是芯片那个自由计数器的原始读数（本模拟器按时间片推进）
    pub fn write(&mut self, addr: u32, value: u32, counter_raw: u32) {
        match addr {
            systimer::COMPARE => self.arm(counter_raw, value),
            _ => {}
        }
    }

    /// 编程一次比较匹配。硬件上比较值是"从计数器清零算起"的绝对时刻，而且之后由硬件
    /// 按同一个值自动重装（全 ROM 只有开机那一次写 `0x82000238 = 0x8072`）。
    ///
    /// TCG 没有周期计数，计数值只跟 guest 跑过多少有关，guest 何时爬到 2.5 ms 纯属偶然，
    /// 所以把计时基准挪到写的这一刻，等价于"固件此刻才让计数器从 0 开始跑"。
    fn arm(&mut self, counter_raw: u32, target: u32) {
        // ⚠ 已知偏差：这里按"1 个计数 = 1/13 µs"用固件写的比较值，得到 2.529 ms 的 tick
        // 周期。C 参考版对同一个源投的是 5 ms —— 但那是它渲染线程里的硬编码
        // （`config.h:19 interruptPeroidms 5` + `main.c:1034 StartInterrupt(2, …)`，
        // 按宿主 CPU 时间计），而且 C **完全不建模比较寄存器**（`0x82000238` 在 C 源码里
        // 出现 0 次）。所以把单位改成 13 MHz/2 只能算"与参考版的硬编码一致"，不是从硬件
        // 或固件读出来的事实。实测这条改动会让开机在片 13306 跑飞（见 README「已知偏差」
        // 与任务 #46），所以先维持现状。
        self.period = target;
        self.epoch = counter_raw;
        self.served = 0;
    }

    /// 换算成"本次计时起点为 0"的计数值
    pub fn counter(&self, counter_raw: u32) -> u32 {
        counter_raw.wrapping_sub(self.epoch)
    }

    /// 固件写帧间隔（FNIT）就是重启 TDMA 时序，帧号从这一刻重新起算
    pub fn rearm_tdma(&mut self) {
        self.tdma_epoch = self.frame;
    }

    /// TDMA 帧号：**每读一次 +1**。
    ///
    /// 固件在这里等的是"帧号等于某个具体值"（`while ((u16)[0x82000000] != 目标)`，
    /// 目标 = 旧帧号 + 2），所以计数器必须逐个值往前爬：按时间片粗步进（一片 2166）会
    /// 直接从目标值上跨过去，循环再也退不出来。逐读 +1 同时满足两条：
    /// 值一定命中，而且和宿主负载无关，运行可复现。
    pub fn next_frame(&mut self) -> u32 {
        self.frame = self.frame.wrapping_add(1);
        self.frame.wrapping_sub(self.tdma_epoch) & 0xffff
    }

    /// 只要跨过了新的周期沿就是挂起状态，**不会因为"投得晚"而失效**：真机上这个源在 AIC 里
    /// 一直挂着，屏蔽期间照常在下个匹配沿拉起。
    ///
    /// 旧模型判的是"计数值落在 compare 之后 8.4 M（≈0.65 s 仿真时间）以内"，实测后果是：
    /// CTIRQ1 一旦被屏蔽超过约 0.65 s 仿真时间就永远回不到窗口里，整次开机的 tick 从此
    /// 归零 —— 表现为 `挡住:位图` 冻结在 1226 之后不再增长、`时钟递增=0`、所有带超时的
    /// 等待永不超时、任务全部排空进 power-off park。
    pub fn matched(&self, counter: u32) -> bool {
        self.period != 0 && counter.wrapping_div(self.period) > self.served
    }

    /// 中断已投递：跳到当前周期。真机上挂起位只有一个，屏蔽期间攒下的匹配沿不补投。
    pub fn consume(&mut self, counter: u32) {
        if self.period != 0 {
            self.served = counter.wrapping_div(self.period);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 钉住当前的计数单位：**1 个计数 = 1/13 µs**，固件写多少就按多少匹配。
    /// 这条不是"正确性"断言，而是"别让它被无声改掉"的断言 —— 已知它比 C 参考版快一倍
    /// （参考版每 5 ms 投 2 号线，见 [`C1xx::arm`] 的注释），改成 13 MHz/2 会让开机跑飞，
    /// 所以真要动这里必须连带解决 #46 并深跑回归。
    #[test]
    fn compare_matches_on_the_raw_13mhz_unit() {
        let mut c = C1xx::default();
        c.write(systimer::COMPARE, 0x8072, 5000);
        assert!(!c.matched(c.counter(5000 + 0x8072 - 1)));
        assert!(c.matched(c.counter(5000 + 0x8072)));
    }

    /// 没编程过比较值就永不匹配。`period != 0` 那个判据是行为，不只是除零保护：
    /// 开机早期固件还没写 `0x82000238`，此时任何计数都不该拉起 CTIRQ1。
    #[test]
    fn unprogrammed_compare_never_matches() {
        let c = C1xx::default();
        assert!(!c.matched(c.counter(9_999_999)));
    }

    /// 投过一次之后，要等下一个完整周期才有新沿；一次跨多个沿也不补投（挂起位只有一个）
    #[test]
    fn one_pending_edge_per_period_regardless_of_how_many_were_crossed() {
        let mut c = C1xx::default();
        let p = 0x8072;
        c.write(systimer::COMPARE, p, 0);
        assert!(c.matched(p + 7 * p), "跨了多个沿当然算挂起");
        c.consume(p + 7 * p);
        assert!(!c.matched(p + 7 * p), "投过之后同一计数上不该还挂着");
        assert!(!c.matched(p + 8 * p - 1), "没到下一个沿不该匹配");
        assert!(c.matched(p + 8 * p));
    }
}
