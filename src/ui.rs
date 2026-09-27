//! UI 线程：开窗、把合成好的帧贴上去、把键盘事件投递给仿真线程。
//!
//! C 版用 SDL2 做同样的三件事；这里换成 winit + softbuffer，好处是不用再带
//! SDL2.dll，也不依赖控制台窗口。

use std::rc::Rc;
use std::sync::MutexGuard;
use std::time::{Duration, Instant};

use softbuffer::{Context, Surface};
use winit::application::ApplicationHandler;
use winit::dpi::PhysicalSize;
use winit::event::{ElementState, KeyEvent, MouseButton, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowId};

use crate::devices::{keypad, keypad::code};
use crate::events::{EventSender, VmEvent};
use crate::frame::Shared;
use crate::pad::Pad;
use crate::vm;

/// 屏幕刷新节奏；固件本身靠中断驱动，这里只是限制重绘频率
const FRAME_INTERVAL: Duration = Duration::from_millis(16);

/// 方向键、回车这类没有字符的键，直接给固件键码
fn named_key(key: &Key) -> Option<u8> {
    Some(match key {
        Key::Named(NamedKey::ArrowUp) => code::UP,
        Key::Named(NamedKey::ArrowDown) => code::DOWN,
        Key::Named(NamedKey::ArrowLeft) => code::LEFT,
        Key::Named(NamedKey::ArrowRight) => code::RIGHT,
        Key::Named(NamedKey::Enter) => code::OK,
        _ => return None,
    })
}

pub fn run(frame: Shared, out: EventSender) {
    let (screen_w, screen_h) = {
        let guard = frame.lock().unwrap_or_else(|p| p.into_inner());
        (guard.width, guard.height)
    };
    let Ok(event_loop) = EventLoop::new() else {
        eprintln!("[ui] 无法创建事件循环，仿真仍在后台线程运行");
        loop {
            std::thread::sleep(FRAME_INTERVAL);
            if vm::STOP.load(std::sync::atomic::Ordering::Relaxed) {
                return;
            }
        }
    };
    event_loop.set_control_flow(ControlFlow::Wait);
    let mut app = App {
        screen_w,
        screen_h,
        pad: Pad::new(screen_w, screen_h),
        frame,
        out,
        window: None,
        context: None,
        surface: None,
        pressed: None,
        cursor: (0.0, 0.0),
        // 帧的 serial 从 0 开始，这里必须是"不可能的值"，否则第一帧会被当成没变化而跳过，
        // 窗口就一直空着不画
        serial: u32::MAX,
    };
    match event_loop.run_app(&mut app) {
        Ok(()) => {}
        Err(e) => eprintln!("[ui] 事件循环结束: {e}"),
    }
}

struct App {
    screen_w: u32,
    screen_h: u32,
    pad: Pad,
    frame: Shared,
    out: EventSender,
    /// 声明顺序影响释放：surface/context 持有 window 的强引用：`surface`/`context` 都持有 window 的强引用
    window: Option<Rc<Window>>,
    context: Option<Context<Rc<Window>>>,
    surface: Option<Surface<Rc<Window>, Rc<Window>>>,
    /// 同一时刻只跟踪一个按键，与 C 版的 `isKeyDown` 行为一致；
    /// 键盘和虚拟按键共用这一个状态，也用它决定面板上哪个键画成"按下"
    pressed: Option<u8>,
    /// 鼠标最近位置：`MouseInput` 不带坐标，命中测试要靠 `CursorMoved` 缓存
    cursor: (f64, f64),
    serial: u32,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let win_h = self.screen_h + crate::pad::height();
        let attrs = Window::default_attributes()
            .with_title("IHD316 (MT6252) Simulator [Rust]  键位: ↑↓←→/wasd 方向  回车/f 确认  q/e 软键  z 拨号  c 挂断/电源")
            .with_inner_size(PhysicalSize::new(self.screen_w, win_h))
            // 面板坐标按物理像素算，禁掉缩放免得点击区域错位
            .with_resizable(false);
        let window = match event_loop.create_window(attrs) {
            Ok(w) => Rc::new(w),
            Err(e) => {
                eprintln!("[ui] 建窗失败: {e}");
                vm::request_stop();
                event_loop.exit();
                return;
            }
        };
        let context = match Context::new(window.clone()) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("[ui] 创建软渲染上下文失败: {e}");
                vm::request_stop();
                event_loop.exit();
                return;
            }
        };
        let mut surface = match Surface::new(&context, window.clone()) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[ui] 创建渲染表面失败: {e}");
                vm::request_stop();
                event_loop.exit();
                return;
            }
        };
        if let (Some(w), Some(h)) = (nonzero(self.screen_w), nonzero(win_h)) {
            if let Err(e) = surface.resize(w, h) {
                eprintln!("[ui] 设置缓冲尺寸失败: {e}");
            }
        }
        self.surface = Some(surface);
        self.context = Some(context);
        self.window = Some(window);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::KeyboardInput { event: KeyEvent { logical_key, state, repeat, .. }, .. } => {
                if repeat {
                    return;
                }
                if matches!(logical_key, Key::Named(NamedKey::Escape)) {
                    vm::request_stop();
                    event_loop.exit();
                    return;
                }
                let text = logical_key.to_text().unwrap_or_default();
                let Some(key) = keypad::key_of(text).or_else(|| named_key(&logical_key)) else {
                    return;
                };
                self.send_key(key, state == ElementState::Pressed);
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.cursor = (position.x, position.y);
            }
            WindowEvent::MouseInput { state, button: MouseButton::Left, .. } => {
                let down = state == ElementState::Pressed;
                let (x, y) = (self.cursor.0 as u32, self.cursor.1 as u32);
                if let Some(key) = self.pad.hit(x, y) {
                    self.send_key(key, down);
                } else if !down {
                    // 在面板外松开：把面板上按着的键补一个抬起，免得卡在按下状态
                    if let Some(held) = self.pressed.take() {
                        let _ = self.out.send(VmEvent::Keyboard { key: held, down: false });
                    }
                }
            }
            WindowEvent::CloseRequested => {
                vm::request_stop();
                event_loop.exit();
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        self.blit();
        event_loop.set_control_flow(ControlFlow::WaitUntil(Instant::now() + FRAME_INTERVAL));
    }
}

impl App {
    /// 把一个按键的按下/松开投给仿真线程。同一时刻只允许一个键被按住。
    fn send_key(&mut self, key: u8, down: bool) {
        match (self.pressed, down) {
            // 松开的一定要是当前按住的那个键，否则丢掉
            (Some(held), false) if held != key => return,
            (Some(held), true) if held == key => return,
            _ => {}
        }
        self.pressed = down.then_some(key);
        if self.out.send(VmEvent::Keyboard { key, down }).is_err() {
            eprintln!("[ui] 仿真线程已结束，按键被丢弃");
        }
        self.paint(true);
    }

    fn blit(&mut self) {
        self.paint(false);
    }

    /// 把 LCD 帧和虚拟面板合成到窗口缓冲。`force` 用于按键按下/松开这种
    /// "固件那一帧没变、但面板要变"的场合。
    fn paint(&mut self, force: bool) {
        let Some(surface) = self.surface.as_mut() else { return };
        let held = self.pressed;
        let guard: MutexGuard<'_, crate::frame::Frame> = self.frame.lock().unwrap_or_else(|p| p.into_inner());
        if !force && guard.serial == self.serial {
            return;
        }
        self.serial = guard.serial;
        let mut buffer = match surface.buffer_mut() {
            Ok(b) => b,
            Err(e) => {
                eprintln!("[ui] 取帧缓冲失败: {e}");
                return;
            }
        };
        let stride = self.screen_w as usize;
        // 先铺面板底色，再把固件那一帧盖在顶部，最后画按键
        for px in buffer.iter_mut() {
            *px = crate::pad::BACKGROUND;
        }
        if stride > 0 {
            let rows = (self.screen_h as usize).min(buffer.len() / stride).min(guard.pixels.len() / stride.max(1));
            for row in 0..rows {
                let src = &guard.pixels[row * stride..(row + 1) * stride];
                buffer[row * stride..(row + 1) * stride].copy_from_slice(src);
            }
        }
        drop(guard);
        self.pad.draw(&mut buffer, stride as u32, held);
        if let Err(e) = buffer.present() {
            eprintln!("[ui] 提交帧失败: {e}");
        }
    }
}

fn nonzero(v: u32) -> Option<std::num::NonZeroU32> {
    std::num::NonZeroU32::new(v)
}
