//! [`Engine`] 是对 `uc_engine` 裸句柄的薄封装。
//!
//! hook 回调里我们先取出裸句柄再借用 `Vm`，否则 `&mut Unicorn` 的借用会和
//! `get_data_mut()` 冲突。所有方法都只在仿真线程上调用。

use std::ffi::c_void;

use unicorn_engine::{
    RegisterARM,
    unicorn_const::{uc_engine, uc_mem_read, uc_mem_write, uc_reg_read, uc_reg_write},
};

#[derive(Clone, Copy)]
pub struct Engine(*mut uc_engine);

// 仿真线程独占；句柄本身不跨线程传递
unsafe impl Send for Engine {}

impl Engine {
    /// # Safety
    /// `raw` 必须是仍然存活的 `uc_open` 返回值
    pub unsafe fn from_raw(raw: *mut uc_engine) -> Self {
        Self(raw)
    }

    fn mem_write<T: Copy>(self, addr: u32, value: T) {
        let err = unsafe {
            uc_mem_write(
                self.0,
                addr as u64,
                &value as *const T as *mut c_void,
                std::mem::size_of::<T>() as u64,
            )
        };
        debug_assert_eq!(err, unicorn_engine::unicorn_const::uc_error::OK, "mem_write {addr:#x}");
    }

    pub fn set_u8(self, addr: u32, v: u8) {
        self.mem_write(addr, v)
    }
    pub fn set_u16(self, addr: u32, v: u16) {
        self.mem_write(addr, v)
    }
    pub fn set_u32(self, addr: u32, v: u32) {
        self.mem_write(addr, v)
    }

    pub fn read_bytes(self, addr: u32, len: usize) -> Vec<u8> {
        let mut buf = vec![0u8; len];
        if len > 0 {
            self.mem_read_slice(addr, &mut buf);
        }
        buf
    }

    /// 探针用的"读不到就算了"版本：`mem_read_slice` 里是 `debug_assert_eq!`，debug 构建下
    /// 拿野指针地址去读会**直接把仿真打崩**。诊断动作必须走这条，不能走 `read_bytes`。
    pub fn try_read(self, addr: u32, len: usize) -> Option<Vec<u8>> {
        if len == 0 {
            return Some(Vec::new());
        }
        let mut buf = vec![0u8; len];
        let err = unsafe {
            uc_mem_read(
                self.0,
                addr as u64,
                buf.as_mut_ptr() as *mut c_void,
                buf.len() as u64,
            )
        };
        (err == unicorn_engine::unicorn_const::uc_error::OK).then_some(buf)
    }

    /// 读一个 guest 字。只用于 RAM 里的内核状态（MMIO 那边有 `Vm::load_u32`）
    pub fn u32(self, addr: u32) -> u32 {
        let b = self.read_bytes(addr, 4);
        u32::from_le_bytes([b[0], b[1], b[2], b[3]])
    }

    pub fn mem_read_slice(self, addr: u32, buf: &mut [u8]) {
        let err = unsafe {
            uc_mem_read(
                self.0,
                addr as u64,
                buf.as_mut_ptr() as *mut c_void,
                buf.len() as u64,
            )
        };
        debug_assert_eq!(err, unicorn_engine::unicorn_const::uc_error::OK, "mem_read {addr:#x}");
    }

    pub fn write_bytes(self, addr: u32, data: &[u8]) {
        let err = unsafe {
            uc_mem_write(
                self.0,
                addr as u64,
                data.as_ptr() as *mut c_void,
                data.len() as u64,
            )
        };
        debug_assert_eq!(err, unicorn_engine::unicorn_const::uc_error::OK, "mem_write {addr:#x}");
    }

    pub fn reg<T: Into<i32>>(self, reg: T) -> u32 {
        let mut v: u32 = 0;
        unsafe { uc_reg_read(self.0, reg.into(), &mut v as *mut u32 as *mut c_void) };
        v
    }

    pub fn set_reg<T: Into<i32>>(self, reg: T, value: u32) {
        unsafe { uc_reg_write(self.0, reg.into(), &value as *const u32 as *mut c_void) };
    }

    pub fn pc(self) -> u32 {
        self.reg(RegisterARM::PC)
    }
    pub fn set_pc(self, v: u32) {
        self.set_reg(RegisterARM::PC, v)
    }
    pub fn lr(self) -> u32 {
        self.reg(RegisterARM::LR)
    }
    pub fn set_lr(self, v: u32) {
        self.set_reg(RegisterARM::LR, v)
    }
    pub fn sp(self) -> u32 {
        self.reg(RegisterARM::SP)
    }
    pub fn cpsr(self) -> u32 {
        self.reg(RegisterARM::CPSR)
    }
    pub fn set_cpsr(self, v: u32) {
        self.set_reg(RegisterARM::CPSR, v)
    }
    /// CPSR bit5：1 表示当前处于 Thumb 态，回填 PC 时要保持低位标记
    pub fn in_thumb(self) -> bool {
        self.cpsr() & 0x20 != 0
    }
}
