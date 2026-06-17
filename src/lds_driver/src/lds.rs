//! LDS02RR / Neato XV-11 binary protocol parser.
//!
//! Ported verbatim from the in-browser decoder in `LDS_Visualizer.html`
//! (itself from kaiaai/LDS):
//!
//!   22-byte packet: FA idx spdL spdM [4x(distL distM sigL sigM)] crcL crcM
//!   90 packets (idx 0xA0..0xF9) x 4 readings = 360 deg. RPM = speed / 64.
//!   distance MSB bit7 = invalid, bit6 = low-signal; distance is the low 14 bits (mm).
//!   A 15-bit checksum over the first 20 bytes is compared to bytes 20..21.

const PKT_LEN: usize = 22;
const CMD: u8 = 0xFA;
const IDX_LO: u8 = 0xA0;
const BAD_MASK: u8 = 0xC0; // invalid (0x80) | strength-warning (0x40)

#[derive(Clone, Copy, Debug)]
pub struct ScanPoint {
    pub angle: u16,    // 0..359 degrees, as reported by the sensor (CW)
    pub dist_mm: u16,  // 0 = invalid / dropped
    pub quality: u16,
}

pub struct LdsParser {
    buf: [u8; PKT_LEN],
    pos: usize,
    finding: bool,
    building: Vec<ScanPoint>,
    pub rpm: f32,
    pub crc_errors: u64,
    pub packets_ok: u64,
}

impl LdsParser {
    pub fn new() -> Self {
        Self {
            buf: [0; PKT_LEN],
            pos: 0,
            finding: true,
            building: Vec::with_capacity(360),
            rpm: 0.0,
            crc_errors: 0,
            packets_ok: 0,
        }
    }

    /// Feed one byte. Returns the just-completed revolution (one Vec per scan)
    /// when the angle wraps back through 0, otherwise None.
    pub fn feed(&mut self, b: u8) -> Option<Vec<ScanPoint>> {
        if self.finding {
            if b == CMD {
                self.buf[0] = b;
                self.pos = 1;
                self.finding = false;
            }
            return None;
        }
        self.buf[self.pos] = b;
        self.pos += 1;
        if self.pos < PKT_LEN {
            return None;
        }
        // full packet collected; resync afterwards regardless of validity
        self.pos = 0;
        self.finding = true;
        if self.valid() {
            self.packets_ok += 1;
            self.parse()
        } else {
            self.crc_errors += 1;
            None
        }
    }

    fn valid(&self) -> bool {
        let p = &self.buf;
        let mut chk32: u32 = 0;
        let mut ix = 0;
        while ix < 20 {
            chk32 = chk32.wrapping_mul(2).wrapping_add(p[ix] as u32 + ((p[ix + 1] as u32) << 8));
            ix += 2;
        }
        let mut cs = (chk32 & 0x7FFF) + (chk32 >> 15);
        cs &= 0x7FFF;
        (cs & 0xFF) as u8 == p[20] && ((cs >> 8) & 0xFF) as u8 == p[21]
    }

    fn parse(&mut self) -> Option<Vec<ScanPoint>> {
        let p = self.buf;
        let start_angle = (p[1].wrapping_sub(IDX_LO)) as u16 * 4;
        self.rpm = (((p[3] as u16) << 8) | p[2] as u16) as f32 / 64.0;

        let mut completed: Option<Vec<ScanPoint>> = None;
        for q in 0..4u16 {
            let o = 4 + (q as usize) * 4;
            let dm = p[o + 1];
            let bad = dm & BAD_MASK;
            let dist = if bad != 0 { 0 } else { p[o] as u16 | (((dm & 0x3F) as u16) << 8) };
            let quality = if bad != 0 { 0 } else { p[o + 2] as u16 | ((p[o + 3] as u16) << 8) };
            let angle = start_angle + q;

            // A revolution boundary is the angle-0 reading: finalize the previous
            // revolution, then start accumulating the new one with this point.
            if angle == 0 && !self.building.is_empty() {
                completed = Some(std::mem::take(&mut self.building));
            }
            self.building.push(ScanPoint { angle, dist_mm: dist, quality });
        }
        completed
    }
}
