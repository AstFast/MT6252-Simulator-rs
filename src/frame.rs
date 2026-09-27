use std::sync::{Arc, Mutex};

/// 合成好的一帧。`serial` 每次更新自增，UI 线程用它判断要不要重绘。
#[derive(Debug)]
pub struct Frame {
    /// 0x00RRGGBB 布局，与 softbuffer 的格式一致
    pub pixels: Vec<u32>,
    pub width: u32,
    pub height: u32,
    pub serial: u32,
}

impl Frame {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            pixels: vec![0xff00_0000; (width * height) as usize],
            width,
            height,
            serial: 0,
        }
    }

    /// 把这一帧写成 PNG（真彩 RGB，无 alpha）。
    ///
    /// 存在的理由只有一个：判断"模拟器窗口里到底显示了什么"必须看**主机合成器**的产物，
    /// 而不是显存 dump。后者要人自己按 `LxWINKEY` 把几层叠回去，而人叠的时候会把图层读反
    /// ——实测就这么误判过一次"叠加层没合成上来"。
    ///
    /// 手写编码器而不引依赖：LZ77 用 deflate 的 stored 块（不压缩，231 KB 量级，够用），
    /// 校验和用 zlib 要求的 adler32。
    pub fn write_png(&self, path: impl AsRef<std::path::Path>) -> std::io::Result<()> {
        let w = self.width as usize;
        let h = self.height as usize;
        // 每行前置一个 filter 字节，0 = None（不做预测，逐行原样）
        let mut raw = Vec::with_capacity(h * (1 + w * 3));
        for y in 0..h {
            raw.push(0);
            for &p in &self.pixels[y * w..(y + 1) * w] {
                raw.push((p >> 16) as u8);
                raw.push((p >> 8) as u8);
                raw.push(p as u8);
            }
        }
        let mut out: Vec<u8> = Vec::with_capacity(raw.len() + 64);
        out.extend_from_slice(b"\x89PNG\r\n\x1a\n");
        chunk(&mut out, b"IHDR", &{
            let mut b = Vec::new();
            b.extend_from_slice(&self.width.to_be_bytes());
            b.extend_from_slice(&self.height.to_be_bytes());
            b.extend_from_slice(&[8, 2, 0, 0, 0]); // 位深 8、色彩类型 2 = RGB、无交错
            b
        });
        chunk(&mut out, b"IDAT", &zlib_stored(&raw));
        chunk(&mut out, b"IEND", &[]);
        std::fs::write(path, out)
    }
}

/// 写一个 PNG 块：长度 + 类型 + 数据 + CRC32（CRC 覆盖类型和数据，不含长度）
fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], body: &[u8]) {
    out.extend_from_slice(&(body.len() as u32).to_be_bytes());
    let mut blob = Vec::with_capacity(4 + body.len());
    blob.extend_from_slice(kind);
    blob.extend_from_slice(body);
    out.extend_from_slice(&blob);
    out.extend_from_slice(&crc32(&blob).to_be_bytes());
}

/// zlib 流 = 2 字节头 + deflate stored 块 + adler32。
/// `0x78 0x01`：CM=8（deflate）、CINFO=7（32 KB 窗口，stored 用不到但合法）、FCHECK 补齐到 31 的倍数
fn zlib_stored(data: &[u8]) -> Vec<u8> {
    let mut out = vec![0x78, 0x01];
    let mut pos = 0;
    loop {
        let n = (data.len() - pos).min(0xFFFF);
        out.push(if pos + n >= data.len() { 1 } else { 0 }); // BFINAL + BTYPE=00
        out.extend_from_slice(&(n as u16).to_le_bytes());
        out.extend_from_slice(&(!(n as u16)).to_le_bytes()); // NLEN
        out.extend_from_slice(&data[pos..pos + n]);
        pos += n;
        if pos >= data.len() {
            break;
        }
    }
    out.extend_from_slice(&adler32(data).to_be_bytes());
    out
}

fn adler32(data: &[u8]) -> u32 {
    const MOD: u32 = 65521;
    let (mut a, mut b) = (1u32, 0u32);
    for &x in data {
        a = (a + x as u32) % MOD;
        b = (b + a) % MOD;
    }
    b << 16 | a
}

/// 表驱动的 CRC-32/PNG（多项式 0xEDB88320，初值 ~0，输出再取反）
fn crc32(data: &[u8]) -> u32 {
    static TABLE: std::sync::OnceLock<[u32; 256]> = std::sync::OnceLock::new();
    let table = TABLE.get_or_init(|| {
        let mut t = [0u32; 256];
        for (i, e) in t.iter_mut().enumerate() {
            let mut c = i as u32;
            for _ in 0..8 {
                c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
            }
            *e = c;
        }
        t
    });
    let mut c = 0xFFFF_FFFFu32;
    for &x in data {
        c = table[((c ^ x as u32) & 0xff) as usize] ^ (c >> 8);
    }
    !c
}

pub type Shared = Arc<Mutex<Frame>>;

pub fn shared(width: u32, height: u32) -> Shared {
    Arc::new(Mutex::new(Frame::new(width, height)))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 两个校验函数各自钉一个公开标准向量：错了 PNG 会被查看器整张拒收，
    /// 而"文件写出来了但打不开"在诊断现场看起来和"画面是黑的"一模一样
    #[test]
    fn checksums_match_their_standard_vectors() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
        assert_eq!(adler32(b"Wikipedia"), 0x11E6_0398);
        assert_eq!(adler32(b""), 1);
    }

    /// stored 块的自检：zlib 头、块头里的 LEN/NLEN 互补、BFINAL 只在最后一块、
    /// 末字节是 adler32 的高字节（数据非空时 a>=1，所以整个校验和不可能全 0）
    #[test]
    fn stored_deflate_blocks_are_well_formed() {
        let data = vec![0xABu8; 70000]; // 逼出第二个块
        let z = zlib_stored(&data);
        assert_eq!(&z[..2], &[0x78, 0x01]);
        let mut pos = 2;
        let mut seen = 0usize;
        let mut blocks = 0;
        for expect_final in [false, true] {
            let final_bit = z[pos] & 1 != 0;
            assert_eq!(final_bit, expect_final, "第 {blocks} 块的 BFINAL 不对");
            assert_eq!(z[pos] >> 1 & 3, 0, "BTYPE 必须是 stored");
            let len = u16::from_le_bytes([z[pos + 1], z[pos + 2]]) as usize;
            let nlen = u16::from_le_bytes([z[pos + 3], z[pos + 4]]) as usize;
            assert_eq!(len ^ nlen, 0xFFFF, "NLEN 不是 LEN 按位取反");
            pos += 5 + len;
            seen += len;
            blocks += 1;
        }
        assert_eq!(seen, data.len(), "两块加起来没覆盖全部数据");
        assert_eq!(blocks, 2);
        assert_eq!(pos, z.len() - 4, "块之后应只剩 4 字节校验和");
        assert_eq!(u32::from_be_bytes(z[pos..pos + 4].try_into().unwrap()), adler32(&data));
    }

    /// 端到端：写出的 PNG 必须能被**独立**地按规范解回来（这里手解，不引解码库），
    /// 且像素值与 `pixels` 一致。图层顺序/色键一旦错，错的像素会在这里被逐字节抓出来
    #[test]
    fn write_png_roundtrips_pixel_values() {
        let mut f = Frame::new(2, 2);
        f.pixels = vec![0x00FF_0000, 0x0000_FF00, 0x0000_00FF, 0x0012_3456];
        let path = std::env::temp_dir().join("mt6252_frame_test_roundtrip.png");
        f.write_png(&path).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let _ = std::fs::remove_file(&path);

        assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n");
        let body = |kind: &[u8; 4]| -> Vec<u8> {
            let at = bytes.windows(4).position(|w| w == kind).unwrap() - 4;
            let len = u32::from_be_bytes(bytes[at..at + 4].try_into().unwrap()) as usize;
            let blob = bytes[at + 4..at + 8 + len].to_vec();
            assert_eq!(
                u32::from_be_bytes(bytes[at + 8 + len..at + 12 + len].try_into().unwrap()),
                crc32(&blob),
                "{:?} 块的 CRC 对不上",
                std::str::from_utf8(kind)
            );
            blob[4..].to_vec()
        };
        let ihdr = body(b"IHDR");
        assert_eq!(&ihdr[..8], &[0, 0, 0, 2, 0, 0, 0, 2]);
        assert_eq!(&ihdr[8..13], &[8, 2, 0, 0, 0], "位深/色彩类型/压缩/滤波/交错");
        assert!(body(b"IEND").is_empty());

        // 手解 IDAT：跳过头和 5 字节块头，去掉每行的 filter 字节
        let idat = body(b"IDAT");
        let mut raw = Vec::new();
        let mut pos = 2;
        while pos < idat.len() - 4 {
            let len = u16::from_le_bytes([idat[pos + 1], idat[pos + 2]]) as usize;
            raw.extend_from_slice(&idat[pos + 5..pos + 5 + len]);
            pos += 5 + len;
        }
        assert_eq!(raw.len(), 2 * (1 + 2 * 3));
        let mut px = Vec::new();
        for y in 0..2 {
            assert_eq!(raw[y * 7], 0, "filter 字节应为 None");
            for x in 0..2 {
                let s = y * 7 + 1 + x * 3;
                px.push((raw[s] as u32) << 16 | (raw[s + 1] as u32) << 8 | raw[s + 2] as u32);
            }
        }
        assert_eq!(px, vec![0x00FF_0000, 0x0000_FF00, 0x0000_00FF, 0x0012_3456]);
    }
}
