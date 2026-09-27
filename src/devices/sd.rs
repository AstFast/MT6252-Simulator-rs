//! MSDC（SD 卡主机控制器）+ 裸扇区镜像文件。
//!
//! 固件的 FAT 驱动把 CMD17/18/24/25 的参数直接当**字节偏移**用（可以从
//! `logs/小说文件读取日志.txt` 里 `read fat32.img(a:c00,p:e820,bc:200)` 看出来），
//! 所以这里不需要再做扇区号换算。

use std::fs::{File, OpenOptions};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::Path;

use crate::memmap::msdc_reg;

/// 写入 SD_CMD_REG 的是"索引 + 标志"的编码值，与固件驱动里的常量一致
#[allow(dead_code)] // SD 协议编码常量镜像，接线到哪条命令随固件版本变化
pub mod cmd {
    pub const CMD2: u32 = 0x0502;
    pub const CMD3_SD: u32 = 0x0303;
    pub const CMD7: u32 = 0x0387;
    pub const CMD8: u32 = 0x0088;
    pub const CMD9: u32 = 0x0109;
    pub const CMD12: u32 = 0x438c;
    pub const CMD13: u32 = 0x008d;
    pub const CMD16: u32 = 0x0090;
    pub const CMD17: u32 = 0x0891;
    pub const CMD18: u32 = 0x1092;
    pub const CMD24: u32 = 0x2898;
    pub const CMD25: u32 = 0x3099;
    pub const CMD41_SD: u32 = 0x01a9;
    pub const CMD55: u32 = 0x00b7;
    pub const ACMD42: u32 = 0x00aa;
    pub const ACMD51: u32 = 0x08b3;
}

/// 数据完成标志
const DAT_STA_DONE: u32 = 0x8000;

#[derive(Debug)]
pub struct SdCard {
    file: Option<File>,
    cmd: u32,
    arg: u32,
    data_stat: u32,
    cmd_stat: u32,
}

impl SdCard {
    pub fn open(path: &Path) -> Self {
        // 镜像被别的进程占用时退化成只读，和 C 版一致
        let file = match OpenOptions::new().read(true).write(true).open(path) {
            Ok(f) => Some(f),
            Err(_) => match File::open(path) {
                Ok(f) => {
                    println!("[sd] {path:?} 已被占用，改用只读方式打开");
                    Some(f)
                }
                Err(_) => {
                    println!("[sd] 没有 SD 卡镜像 {path:?}，跳过加载");
                    None
                }
            },
        };
        Self { file, cmd: 0, arg: 0, data_stat: 0, cmd_stat: 0 }
    }

    pub fn present(&self) -> bool {
        self.file.is_some()
    }

    pub fn cmd(&self) -> u32 {
        self.cmd
    }

    /// SD_ARG_REG：对读写命令而言就是镜像内的字节偏移
    pub fn arg(&self) -> u32 {
        self.arg
    }

    pub fn write(&mut self, addr: u32, value: u32) {
        match addr {
            msdc_reg::CMD => self.cmd = value & 0xffff,
            msdc_reg::ARG => self.arg = value,
            msdc_reg::DATA_STAT => self.data_stat = value,
            msdc_reg::CMD_STAT => self.cmd_stat = value,
            msdc_reg::DAT_STA => self.data_stat = value,
            _ => {}
        }
    }

    pub fn read(&mut self, addr: u32) -> Option<u32> {
        let value = match addr {
            // 1 = 命令响应成功，2 = 超时，4 = CRC 错
            msdc_reg::CMD_STAT => 1,
            msdc_reg::DATA_STAT => self.data_stat,
            msdc_reg::DAT_STA => self.data_stat,
            msdc_reg::DATA_RESP0 => self.resp_word(0, true),
            msdc_reg::DATA_RESP1 => self.resp_word(1, true),
            msdc_reg::DATA_RESP2 => self.resp_word(2, true),
            msdc_reg::DATA_RESP3 => self.resp_word(3, true),
            msdc_reg::CMD_RESP0 => self.resp_word(0, false),
            msdc_reg::CMD_RESP1 => self.resp_word(1, false),
            msdc_reg::CMD_RESP2 => self.resp_word(2, false),
            msdc_reg::CMD_RESP3 => self.resp_word(3, false),
            _ => return None,
        };
        Some(value)
    }

    /// 按当前命令返回 CID/CSD/OCR 等响应字。`data` 区分数据响应寄存器和命令响应寄存器。
    fn resp_word(&self, index: usize, data: bool) -> u32 {
        match (data, index, self.cmd) {
            (true, 0, cmd::CMD2) | (false, 0, cmd::CMD2) => 0xF016_C1C4,
            (true, 1, cmd::CMD2) | (false, 1, cmd::CMD2) => 0x77,
            (true, 2, cmd::CMD2) | (false, 2, cmd::CMD2) => 0,
            (true, 3, cmd::CMD2) | (false, 3, cmd::CMD2) => 3,
            (true, _, cmd::CMD8) => 0x1aa,
            // CSD v1（RESP0=CSD[127:96] … RESP3=CSD[31:0]），手搓的常量，字段并不自洽：
            // `READ_BL_LEN`（CSD[49:46] = RESP2 bits[17:14]）这里是 4（16 字节），不是 9。
            // **但把它改成 9 实测毫无效果**（1200 片轨迹逐字节相同，片 682 那次 −34 卸载照旧），
            // 所以"扇区大小来自 CSD 的 READ_BL_LEN"这条推断已被否证，别再从这里下手。
            // 真正的 −34 出自 FAT 作业处理内部，见 README「第十三续」。
            (true, 0, cmd::CMD9) => 0x0000_e004,
            (true, 1, cmd::CMD9) => 0x000f_f577,
            (true, 2, cmd::CMD9) => 0x0009_0ff7,
            (true, 3, cmd::CMD9) => 0x0000_04a0,
            (true, _, cmd::CMD55) => 0x20,
            // bit31=1 就绪，bit23-15=0xFF 表示 2.7~3.6V，标准容量卡
            (true, _, cmd::CMD41_SD) => 0x80FF_8000,
            (true, _, cmd::CMD3_SD) => 0x3001,
            // R1 的 `CURRENT_STATE[12:9]` 必须是 **4 = TRANST（传态）**，光给 bit8 不够。
            // 驱动每条 CMD13 之后比的就是这一位段：`0x0816_023E lsls r0,#0x13 ; lsrs r0,#0x1c ;
            // cmp r0,#4`（环在 `0x0816_020C`，计数 256）。原来答 `0x100` 时状态是 0，
            // 于是**每读一个块都耗尽 256 次 CMD13**（实测 1200 片里 2049 条，分段
            // `[1,256,256,256,256,256,256,256,256]`，`dump/sdseq.log`）。
            // 注意 `0x82B6_226 lsrs r0,r4,#0x10 ; beq` 要求 R1[31:16] == 0，所以不能顺手
            // 把状态塞进高位；耗尽虽然也返回成功（`0x0816_024A movs r0,#0`），但白烧时间。
            // 0x900 = bit8 READY_FOR_DATA | (4 << 9) CURRENT_STATE=TRANST。
            (true, _, cmd::CMD13) | (false, _, cmd::CMD13) => 0x900,
            (true, _, cmd::ACMD51) => 0,
            _ => 0,
        }
    }

    /// 读镜像到 `buf`，返回实际读到的长度
    pub fn read_at(&mut self, offset: u32, count: usize) -> Option<Vec<u8>> {
        let mut buf = vec![0u8; count];
        let file = self.file.as_mut()?;
        if file.seek(SeekFrom::Start(offset as u64)).is_err() {
            println!("[sd] 移动文件指针失败 offset={offset:#x}");
            return None;
        }
        match file.read(&mut buf) {
            Ok(n) if n == count => Some(buf),
            Ok(n) => {
                println!("[sd] 读镜像不完整 {n}/{count} offset={offset:#x}");
                None
            }
            Err(e) => {
                println!("[sd] 读镜像失败 offset={offset:#x}: {e}");
                None
            }
        }
    }

    pub fn write_at(&mut self, offset: u32, data: &[u8]) -> bool {
        let Some(file) = self.file.as_mut() else { return false };
        if file.seek(SeekFrom::Start(offset as u64)).is_err() || file.write_all(data).is_err() {
            println!("[sd] 写镜像失败 offset={offset:#x}");
            return false;
        }
        true
    }

    /// 一次数据传输完成
    pub fn finish_transfer(&mut self) {
        self.data_stat = DAT_STA_DONE;
    }
}
