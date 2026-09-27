//! 外设模型。每个设备的寄存器状态由自身持有，绝不写回 MMIO 地址
//! （MMIO 地址的写入会再次触发自己的回调，形成递归）。

pub mod c1xx;
pub mod dma;
pub mod irqc;
pub mod keypad;
pub mod l1_win;
pub mod lcd;
pub mod rtc;
pub mod sd;
pub mod sfi;
pub mod sim;
