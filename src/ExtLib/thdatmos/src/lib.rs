//! thdatmos — real-time TrueHD Atmos decoding (using the `truehd` library from
//! the truehdd project), exposing through a C API the samples of each element
//! (bed channel or object) with its position, gain and state.
//! E-AC-3 (with or without Atmos) is decoded with OpenJOC, see `eac3.rs`.

pub mod eac3;

use std::collections::VecDeque;
use std::ffi::{CString, c_char};
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::PathBuf;

use matroska_demuxer::{Frame as MkvFrame, MatroskaFile};
use truehd::process::MAX_PRESENTATIONS;
use truehd::process::decode::{DecodedAccessUnit, Decoder};
use truehd::process::extract::Extractor;
use truehd::process::parse::Parser;
use truehd::structs::channel::ChannelLabel;
use truehd::structs::oamd::{GAIN_MINUS_INFINITY, ObjectAudioMetadataPayload, SpeakerLabels};

pub const MAX_CH: usize = 16;
pub const MAX_SAMPLES: usize = 160;

/// Element kind.
pub const KIND_OBJECT: u32 = 0; // object (or bed channel placed at its speaker)
pub const KIND_LFE: u32 = 1; // LFE channel

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct ThdElement {
    pub kind: u32,
    pub active: u32,
    /// DAMF position: x left(-1)/right(+1), y back(-1)/front(+1), z floor(0)/ceiling(1).
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Linear gain.
    pub gain: f32,
}

#[repr(C)]
pub struct ThdFrame {
    /// Position of the first sample, in samples from the start of the stream.
    pub sample_pos: u64,
    pub samples: u32,
    pub channels: u32,
    /// Sample rate (Hz).
    pub sample_rate: u32,
    /// Float PCM [-1, 1], interleaved: pcm[sample][element].
    pub pcm: [[f32; MAX_CH]; MAX_SAMPLES],
    pub elements: [ThdElement; MAX_CH],
}

enum Source {
    Raw(BufReader<File>),
    Mkv {
        mkv: Box<MatroskaFile<BufReader<File>>>,
        track: u64,
        frame: MkvFrame,
    },
}

impl Source {
    fn open(path: PathBuf) -> Result<Self, String> {
        let is_mkv = path
            .extension()
            .map(|e| {
                let e = e.to_string_lossy().to_ascii_lowercase();
                e == "mkv" || e == "mka" || e == "mk3d"
            })
            .unwrap_or(false);
        let file = File::open(&path).map_err(|e| format!("cannot open file: {e}"))?;
        let reader = BufReader::with_capacity(1 << 20, file);
        if !is_mkv {
            return Ok(Source::Raw(reader));
        }
        let mkv = MatroskaFile::open(reader).map_err(|e| format!("unreadable MKV: {e}"))?;
        let track = mkv
            .tracks()
            .iter()
            .find(|t| t.codec_id() == "A_TRUEHD")
            .map(|t| t.track_number().get())
            .ok_or_else(|| "no TrueHD track in this file".to_string())?;
        Ok(Source::Mkv { mkv: Box::new(mkv), track, frame: MkvFrame::default() })
    }

    /// Returns the next chunk of TrueHD stream, or false at end of file.
    fn next_chunk(&mut self, buf: &mut Vec<u8>) -> Result<bool, String> {
        buf.clear();
        match self {
            Source::Raw(r) => {
                buf.resize(64 * 1024, 0);
                let n = r.read(buf).map_err(|e| e.to_string())?;
                buf.truncate(n);
                Ok(n > 0)
            }
            Source::Mkv { mkv, track, frame } => loop {
                match mkv.next_frame(frame) {
                    Ok(true) => {
                        if frame.track == *track {
                            buf.extend_from_slice(&frame.data);
                            return Ok(true);
                        }
                    }
                    Ok(false) => return Ok(false),
                    Err(e) => return Err(format!("MKV read error: {e}")),
                }
            },
        }
    }
}

struct Update {
    pos: u64,
    index: usize,
    element: ThdElement,
}

pub struct Ctx {
    source: Option<Source>,
    mat: MatUnpacker,
    chunk: Vec<u8>,
    eof: bool,
    extractor: Extractor,
    parser: Parser,
    decoder: Decoder,
    required: [bool; MAX_PRESENTATIONS],
    total: u64,
    elements: [ThdElement; MAX_CH],
    pending: Vec<Update>,
    queue: VecDeque<Box<ThdFrame>>,
    atmos_seen: bool,
    eac3: eac3::Eac3Decoder,
    eac3_seen: bool,
    pub last_error: Option<CString>,
}

impl Ctx {
    pub fn open(path: PathBuf) -> Result<Self, String> {
        let mut c = Self::new();
        c.source = Some(Source::open(path)?);
        Ok(c)
    }

    pub fn new() -> Self {
        let mut required = [false; MAX_PRESENTATIONS];
        required[0] = true; // 2 channels (stereo-only tracks)
        required[1] = true; // 6 channels (5.1 tracks have no 8-channel presentation)
        required[2] = true; // 8 channels (non-Atmos fallback)
        required[3] = true; // Atmos (16 elements)
        let mut parser = Parser::default();
        parser.set_required_presentations(&required);
        Ctx {
            source: None,
            mat: MatUnpacker::default(),
            chunk: Vec::new(),
            eof: false,
            extractor: Extractor::default(),
            parser,
            decoder: Decoder::default(),
            required,
            total: 0,
            elements: [ThdElement::default(); MAX_CH],
            pending: Vec::new(),
            queue: VecDeque::new(),
            atmos_seen: false,
            eac3: eac3::Eac3Decoder::new(),
            eac3_seen: false,
            last_error: None,
        }
    }

    /// Resets the decoder (seek, flush).
    pub fn reset(&mut self) {
        let required = self.required;
        let mut parser = Parser::default();
        parser.set_required_presentations(&required);
        self.parser = parser;
        self.decoder = Decoder::default();
        self.extractor = Extractor::default();
        self.mat = MatUnpacker::default();
        self.pending.clear();
        self.queue.clear();
        self.elements = [ThdElement::default(); MAX_CH];
        self.total = 0;
        self.atmos_seen = false;
        self.eac3.reset();
    }

    /// Pushes raw TrueHD stream bytes.
    pub fn push(&mut self, data: &[u8]) {
        self.extractor.push_bytes(data);
        self.drain_extractor();
    }

    /// Pushes one IEC 61937 burst (TrueHD packed in a MAT frame, as produced by the
    /// MPC-BE audio decoder). Returns false if the burst is not TrueHD.
    /// Also accepts E-AC-3 bursts (Dolby Digital Plus, with or without Atmos).
    pub fn push_iec61937(&mut self, burst: &[u8]) -> bool {
        let mut out = Vec::with_capacity(MAT_SIZE);
        if self.mat.unpack(burst, &mut out) {
            self.push(&out);
            return true;
        }
        out.clear();
        if eac3::unpack_iec61937(burst, &mut out) {
            self.eac3_seen = true;
            let mut blocks = Vec::new();
            self.eac3.push(&out, &mut blocks);
            for b in blocks {
                self.queue_block(&b);
            }
            if let Some(e) = self.eac3.last_error.take() {
                self.last_error = CString::new(e).ok();
            }
            return true;
        }
        false
    }

    /// Stream information: bit 0 TrueHD, bit 1 E-AC-3, bit 2 Atmos objects.
    pub fn info(&self) -> u32 {
        let mut i = 0;
        if self.eac3_seen {
            i |= 2;
            if self.eac3.is_atmos() {
                i |= 4;
            }
        } else if self.total > 0 {
            i |= 1;
            if self.atmos_seen {
                i |= 4;
            }
        }
        i
    }

    /// Splits a decoded E-AC-3 block into frames.
    fn queue_block(&mut self, b: &eac3::Block) {
        let ch = b.channels.min(MAX_CH);
        let mut done = 0;
        let mut elements = b.elements;
        while done < b.samples {
            let n = (b.samples - done).min(MAX_SAMPLES);
            // Element updates up to the start of this frame.
            for (offset, e) in &b.changes {
                if *offset <= done {
                    elements = *e;
                }
            }
            let mut f = Box::new(ThdFrame {
                sample_pos: self.total,
                samples: n as u32,
                channels: ch as u32,
                sample_rate: b.sample_rate,
                pcm: [[0.0; MAX_CH]; MAX_SAMPLES],
                elements,
            });
            for s in 0..n {
                let row = &b.pcm[(done + s) * b.channels..];
                f.pcm[s][..ch].copy_from_slice(&row[..ch]);
            }
            self.total += n as u64;
            self.queue.push_back(f);
            done += n;
        }
    }

    /// Next decoded frame, if any.
    pub fn pull(&mut self) -> Option<Box<ThdFrame>> {
        self.queue.pop_front()
    }

    /// Returns the next decoded frame (Ok(None) at end of stream). File mode.
    pub fn read(&mut self) -> Result<Option<Box<ThdFrame>>, String> {
        while self.queue.is_empty() {
            if self.eof {
                return Ok(None);
            }
            let Some(src) = self.source.as_mut() else { return Ok(None) };
            if !src.next_chunk(&mut self.chunk)? {
                self.eof = true;
                continue;
            }
            let chunk = std::mem::take(&mut self.chunk);
            self.push(&chunk);
            self.chunk = chunk;
        }
        Ok(self.queue.pop_front())
    }

    fn drain_extractor(&mut self) {
        while let Some(res) = self.extractor.next() {
            let frame = match res {
                Ok(f) => f,
                Err(truehd::utils::errors::ExtractError::InsufficientData) => break,
                Err(_) => continue, // the extractor resyncs by itself
            };
            let au = match self.parser.parse(&frame) {
                Ok(au) => au,
                Err(_) => {
                    self.parser.reset_for_next_major_sync();
                    self.decoder.reset_for_next_major_sync();
                    continue;
                }
            };
            match self.decoder.decode_presentations(&au, &self.required) {
                Ok(mut res) => {
                    // Richest presentation present: Atmos, then 7.1, 5.1, stereo.
                    let d = res[3].take().or_else(|| res[2].take()).or_else(|| res[1].take()).or_else(|| res[0].take());
                    if let Some(d) = d {
                        self.handle(d);
                    }
                }
                Err(_) => self.decoder.reset_for_next_major_sync(),
            }
        }
    }

    fn handle(&mut self, d: DecodedAccessUnit) {
        if d.is_duplicate || d.sample_length == 0 {
            return;
        }
        for oamd in &d.oamd {
            self.atmos_seen = true;
            self.queue_oamd(oamd);
        }
        let n = d.sample_length.min(MAX_SAMPLES);
        let end = self.total + n as u64;

        // Apply metadata updates due before the end of this frame.
        let mut i = 0;
        while i < self.pending.len() {
            if self.pending[i].pos < end {
                let u = self.pending.remove(i);
                self.elements[u.index] = u.element;
            } else {
                i += 1;
            }
        }

        let ch = d.channel_count.min(MAX_CH);
        if d.oamd.is_empty() && !self.atmos_seen {
            // Non-Atmos track: each channel becomes a fixed object at its speaker position.
            for (c, l) in d.channel_labels.iter().take(ch).enumerate() {
                self.elements[c] = channel_element(l);
            }
        }
        let mut f = Box::new(ThdFrame {
            sample_pos: self.total,
            samples: n as u32,
            channels: ch as u32,
            sample_rate: d.sampling_frequency,
            pcm: [[0.0; MAX_CH]; MAX_SAMPLES],
            elements: self.elements,
        });
        const SCALE: f32 = 1.0 / 8_388_608.0; // 2^23
        for s in 0..n {
            for c in 0..ch {
                f.pcm[s][c] = d.pcm_data[s][c] as f32 * SCALE;
            }
        }
        self.total = end;
        self.queue.push_back(f);
    }

    fn queue_oamd(&mut self, oamd: &ObjectAudioMetadataPayload) {
        let Some(oe) = &oamd.object_element else { return };
        let Some(block) = oe.md_update_info.block_update_info.first() else { return };
        let pos = self.total
            + oe.md_update_info.sample_offset as u64
            + u64::from(block.block_offset_factor_bits) * 32
            + oamd.evo_sample_offset;
        let pos_vec = oamd.get_damf_pos();
        let bed = oamd
            .program_assignment
            .bed_assignment
            .first()
            .map(|b| b.to_index_vec())
            .unwrap_or_default();

        for i in 0..oamd.object_count.min(MAX_CH) {
            let Some(od) = oe.object_data.get(i).and_then(|v| v.first()) else { continue };
            let basic = &od.object_basic_info;
            let gain = if basic.object_gain == GAIN_MINUS_INFINITY {
                0.0
            } else {
                10f32.powf(basic.object_gain as f32 / 20.0)
            };
            let mut e = ThdElement {
                kind: KIND_OBJECT,
                active: (!od.b_object_not_active) as u32,
                gain,
                ..Default::default()
            };
            if od.b_object_in_bed_or_isf {
                let label = bed.get(i).and_then(|&b| SpeakerLabels::from_u8(b as u8));
                match label {
                    Some(SpeakerLabels::LFE) | Some(SpeakerLabels::LFE2) => e.kind = KIND_LFE,
                    Some(l) => {
                        let p = l.pos();
                        (e.x, e.y, e.z) = (p[0], p[1], p[2].max(0.0));
                    }
                    None => {}
                }
            } else if let Some(p) = pos_vec.get(i).and_then(|v| v.first()) {
                (e.x, e.y, e.z) = (p[0] as f32, p[1] as f32, p[2] as f32);
            }
            self.pending.push(Update { pos, index: i, element: e });
        }
    }
}

fn channel_element(l: &ChannelLabel) -> ThdElement {
    use ChannelLabel::*;
    let (x, y, z) = match l {
        LFE | LFE2 => {
            return ThdElement { kind: KIND_LFE, active: 1, gain: 1.0, ..Default::default() };
        }
        L => (-1.0, 1.0, 0.0),
        R => (1.0, 1.0, 0.0),
        C => (0.0, 1.0, 0.0),
        Ls => (-1.0, 0.0, 0.0),
        Rs => (1.0, 0.0, 0.0),
        Lb => (-1.0, -1.0, 0.0),
        Rb => (1.0, -1.0, 0.0),
        Cb => (0.0, -1.0, 0.0),
        Lsc => (-0.5, 1.0, 0.0),
        Rsc => (0.5, 1.0, 0.0),
        Lsd => (-1.0, -0.5, 0.0),
        Rsd => (1.0, -0.5, 0.0),
        Lw => (-1.0, 0.68, 0.0),
        Rw => (1.0, 0.68, 0.0),
        Tfl => (-1.0, 1.0, 1.0),
        Tfr => (1.0, 1.0, 1.0),
        Tsl => (-1.0, 0.0, 1.0),
        Tsr => (1.0, 0.0, 1.0),
        Tbl => (-1.0, -1.0, 1.0),
        Tbr => (1.0, -1.0, 1.0),
        Tc => (0.0, 0.0, 1.0),
        Tfc => (0.0, 1.0, 1.0),
    };
    ThdElement { kind: KIND_OBJECT, active: 1, x, y, z, gain: 1.0 }
}

// ------------------------------------------------------- MAT unpacking ---

/// Size of an IEC 61937 TrueHD burst (8-byte header + MAT frame).
pub const MAT_SIZE: usize = 61440;
const IEC_HEADER: usize = 8;
const MAT_START_END: usize = IEC_HEADER + 20; // end of the MAT start code
const MAT_MIDDLE: usize = 30708 + IEC_HEADER; // position of the middle code
const MAT_MIDDLE_END: usize = MAT_MIDDLE + 12;
const MAT_LIMIT: usize = MAT_SIZE - 24; // start of the end code
const IEC61937_TRUEHD: u8 = 0x16;

/// Rebuilds the TrueHD stream from MAT frames: removes the IEC header, the byte
/// swapping, the MAT codes and the padding between access units.
#[derive(Default)]
struct MatUnpacker {
    carry: Vec<u8>, // pending payload (incomplete access unit)
}

impl MatUnpacker {
    fn unpack(&mut self, burst: &[u8], out: &mut Vec<u8>) -> bool {
        if burst.len() < MAT_LIMIT {
            return false;
        }
        // IEC 61937 header: little-endian 16-bit words Pa=F872, Pb=4E1F, Pc=type.
        if burst[0] != 0x72 || burst[1] != 0xF8 || burst[2] != 0x1F || burst[3] != 0x4E || burst[4] != IEC61937_TRUEHD {
            return false;
        }
        // Payload is byte-swapped in pairs.
        let mut swapped = vec![0u8; MAT_LIMIT];
        let mut i = IEC_HEADER;
        while i + 1 < MAT_LIMIT {
            swapped[i] = burst[i + 1];
            swapped[i + 1] = burst[i];
            i += 2;
        }
        self.carry.extend_from_slice(&swapped[MAT_START_END..MAT_MIDDLE]);
        self.carry.extend_from_slice(&swapped[MAT_MIDDLE_END..MAT_LIMIT]);

        // Split into access units; zero words are padding.
        let buf = &self.carry;
        let mut p = 0;
        while p + 2 <= buf.len() {
            let w = u16::from_be_bytes([buf[p], buf[p + 1]]);
            if w == 0 {
                p += 2;
                continue;
            }
            let len = ((w & 0x0FFF) as usize) * 2;
            if len < 4 {
                p += 2; // inconsistent: skip, the extractor will resync
                continue;
            }
            if p + len > buf.len() {
                break; // unit continues in the next MAT frame
            }
            out.extend_from_slice(&buf[p..p + len]);
            p += len;
        }
        self.carry.drain(..p);
        if self.carry.len() > MAT_SIZE * 4 {
            self.carry.clear();
        }
        true
    }
}

// ------------------------------------------------------------------ C API ---

/// Creates a decoder fed by thd_push / thd_push_iec61937.
#[unsafe(no_mangle)]
pub extern "C" fn thd_create() -> *mut Ctx {
    Box::into_raw(Box::new(Ctx::new()))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn thd_reset(ctx: *mut Ctx) {
    unsafe { &mut *ctx }.reset();
}

/// Raw TrueHD stream.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn thd_push(ctx: *mut Ctx, data: *const u8, len: usize) {
    let ctx = unsafe { &mut *ctx };
    ctx.push(unsafe { std::slice::from_raw_parts(data, len) });
}

/// IEC 61937 burst: TrueHD (MAT frame) or E-AC-3. Returns 1 if accepted, 0 otherwise.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn thd_push_iec61937(ctx: *mut Ctx, data: *const u8, len: usize) -> i32 {
    let ctx = unsafe { &mut *ctx };
    ctx.push_iec61937(unsafe { std::slice::from_raw_parts(data, len) }) as i32
}

/// Stream information: bit 0 TrueHD, bit 1 E-AC-3, bit 2 Atmos objects.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn thd_info(ctx: *const Ctx) -> u32 {
    unsafe { &*ctx }.info()
}

/// Next decoded frame: 1 if `out` was filled, 0 if more data is needed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn thd_pull(ctx: *mut Ctx, out: *mut ThdFrame) -> i32 {
    let ctx = unsafe { &mut *ctx };
    match ctx.pull() {
        Some(f) => {
            unsafe { std::ptr::copy_nonoverlapping(&*f as *const ThdFrame, out, 1) };
            1
        }
        None => 0,
    }
}


/// Opens a .mkv/.mka file (first TrueHD track) or a raw .thd stream.
/// `path`: zero-terminated UTF-16 path. Returns NULL on failure
/// (message in `err`, buffer of `err_len` bytes, UTF-8).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn thd_open(path: *const u16, err: *mut c_char, err_len: usize) -> *mut Ctx {
    let path = unsafe {
        let mut n = 0;
        while *path.add(n) != 0 {
            n += 1;
        }
        String::from_utf16_lossy(std::slice::from_raw_parts(path, n))
    };
    match Ctx::open(PathBuf::from(path)) {
        Ok(c) => Box::into_raw(Box::new(c)),
        Err(e) => {
            unsafe { write_err(&e, err, err_len) };
            std::ptr::null_mut()
        }
    }
}

/// Reads the next frame into `out`. Returns 1 (frame), 0 (end), -1 (error, see thd_error).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn thd_read(ctx: *mut Ctx, out: *mut ThdFrame) -> i32 {
    let ctx = unsafe { &mut *ctx };
    match ctx.read() {
        Ok(Some(f)) => {
            unsafe { std::ptr::copy_nonoverlapping(&*f as *const ThdFrame, out, 1) };
            1
        }
        Ok(None) => 0,
        Err(e) => {
            ctx.last_error = CString::new(e).ok();
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn thd_error(ctx: *const Ctx) -> *const c_char {
    let ctx = unsafe { &*ctx };
    ctx.last_error.as_ref().map(|s| s.as_ptr()).unwrap_or(c"".as_ptr())
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn thd_close(ctx: *mut Ctx) {
    if !ctx.is_null() {
        drop(unsafe { Box::from_raw(ctx) });
    }
}

unsafe fn write_err(msg: &str, err: *mut c_char, len: usize) {
    if err.is_null() || len == 0 {
        return;
    }
    let b = msg.as_bytes();
    let n = b.len().min(len - 1);
    unsafe {
        std::ptr::copy_nonoverlapping(b.as_ptr() as *const c_char, err, n);
        *err.add(n) = 0;
    }
}
