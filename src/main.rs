//! MT6252（联发科 2G 基带）固件模拟器，Rust + Unicorn 重写版。
//!
//! 线程模型只有两条：仿真线程独占 `Unicorn` 句柄（Unicorn 不是线程安全的），
//! UI 线程只管窗口和帧缓冲；两者通过 mpsc 事件通道和 `Arc<Mutex<Frame>>` 交互。
//!
//! 芯片行为在 `memmap.rs` + `devices/` 里，跟具体固件绑定的一切都在 `profiles/*.toml`。

mod config;
mod devices;
mod engine;
mod events;
mod frame;
/// 数据手册/C 版 `defined.h` 的寄存器镜像，允许暂时没接线的位段存在
#[allow(dead_code)]
mod memmap;
mod pad;
mod profile;
mod ui;
mod vm;

use std::path::{Path, PathBuf};
use std::thread;

use crate::profile::Profile;
use crate::vm::Vm;

fn main() {
    init_console();
    println!("MT6252 固件模拟器（Rust 重写版）");

    let (profile, rom_path, rom_bytes) = match startup() {
        Ok(pair) => pair,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    let (width, height) = (profile.lcd.width, profile.lcd.height);
    println!("固件: {} / {}x{}", rom_path.display(), width, height);
    println!("SD : {}", config::sdcard_image().display());

    let (tx, rx) = events::channel();
    let frame = frame::shared(width, height);
    let ui_frame = frame.clone();
    let emu_tx = tx.clone();

    let emu = thread::Builder::new()
        .name("emu".into())
        .spawn(move || {
            let vm = Vm::new(rx, ui_frame, profile);
            if let Err(e) = vm.boot(rom_bytes, &emu_tx) {
                eprintln!("[emu] 仿真退出: {e}");
                vm::request_stop();
            }
        })
        .expect("启动仿真线程");

    // `MT6252_NO_UI=1`：不开窗口，纯测量用。有界跑靠 `MT6252_MAX_SLICES` 自己收工，
    // 但 GUI 那种跑法永远不会退出，所以计数只能在心跳里打（见 `Vm::print_counters`）。
    if crate::config::env_on("MT6252_NO_UI") {
        let _ = emu.join();
        vm::request_stop();
    } else {
        ui::run(frame, tx);
        vm::request_stop();
        let _ = emu.join();
    }
    println!("已退出");
}

/// 读镜像 → 按 CRC 认画像 → 若画像指定了别的文件名则以画像为准重新读
fn startup() -> Result<(Profile, PathBuf, Vec<u8>), String> {
    let mut path = match std::env::var("MT6252_ROM") {
        Ok(dir) => PathBuf::from(dir),
        Err(_) => config::asset("08000000.bin"),
    };
    let mut bytes = std::fs::read(&path).map_err(|e| format!("读固件 {path:?} 失败: {e}"))?;
    let profile = Profile::load(&bytes, &file_name(&path))?;
    if profile::validate(&profile).is_err() {
        return Err(format!("画像 {} 校验未通过", profile.name));
    }
    if profile.rom.file != file_name(&path) {
        path = config::asset(&profile.rom.file);
        bytes = std::fs::read(&path).map_err(|e| format!("读固件 {path:?} 失败: {e}"))?;
    }
    Ok((profile, path, bytes))
}

fn file_name(path: &Path) -> String {
    path.file_name().and_then(|n| n.to_str()).unwrap_or_default().to_owned()
}

/// 控制台里中文日志需要 UTF-8 代码页
#[cfg(windows)]
fn init_console() {
    unsafe extern "system" {
        fn SetConsoleOutputCP(code_page: u32) -> i32;
    }
    unsafe {
        SetConsoleOutputCP(65001);
    }
}

#[cfg(not(windows))]
fn init_console() {}
