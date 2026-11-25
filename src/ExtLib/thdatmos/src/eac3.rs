//! E-AC-3 (Dolby Digital Plus) decoding with OpenJOC.
//!
//! - E-AC-3 with Atmos (JOC): OpenJOC reconstructs the objects; each one is passed
//!   on as an object with its own moving position (no intermediate speaker render),
//!   and the AVR renders them to the real speakers, as with a native Atmos stream.
//! - E-AC-3 without Atmos: decoded by the OpenJOC E-AC-3 core decoder; each channel
//!   becomes a fixed object, like a non-Atmos TrueHD track.

use crate::{KIND_LFE, KIND_OBJECT, MAX_CH, ThdElement};
use openjoc_api::{OpenJocConfig, OpenJocError, OpenJocPacket, OpenJocSession};
use openjoc_eac3::{
    AccessUnitParse, ChannelLocation, JocAccessUnitPcmDecoder, StreamType, group_access_units, index_syncframes,
    parse_access_unit_bounds, parse_syncframe_header,
};

const IEC61937_EAC3: u16 = 0x15;
// Object output needs a layout with 3 coordinate dimensions (heights).
const RENDER_LAYOUT: &str = "9.1.6";
const MAX_BUFFER: usize = 1 << 20; // resync if this much data never forms an access unit

/// One block of decoded audio: interleaved PCM and the element of each channel.
/// `changes` holds later element updates within the block (sample offset, elements).
pub struct Block {
    pub pcm: Vec<f32>,
    pub channels: usize,
    pub samples: usize,
    pub sample_rate: u32,
    pub elements: [ThdElement; MAX_CH],
    pub changes: Vec<(usize, [ThdElement; MAX_CH])>,
}

/// Extracts the E-AC-3 payload of one IEC 61937 burst (16-bit words, byte-swapped).
/// Returns false if the burst is not E-AC-3.
pub fn unpack_iec61937(burst: &[u8], out: &mut Vec<u8>) -> bool {
    if burst.len() < 8 {
        return false;
    }
    let w = |i: usize| u16::from_le_bytes([burst[i], burst[i + 1]]);
    if w(0) != 0xF872 || w(2) != 0x4E1F || (w(4) & 0x7F) != IEC61937_EAC3 {
        return false;
    }
    let len = (w(6) as usize).min(burst.len() - 8);
    let even = (len + 1).min(burst.len() - 8) & !1;
    let start = out.len();
    for pair in burst[8..8 + even].chunks_exact(2) {
        out.push(pair[1]);
        out.push(pair[0]);
    }
    out.truncate(start + len);
    // Packers that do not swap: the syncword then reads 77 0B.
    if out.len() >= start + 2 && out[start] == 0x77 && out[start + 1] == 0x0B {
        for pair in out[start..].chunks_exact_mut(2) {
            pair.swap(0, 1);
        }
    }
    true
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Unknown,
    Joc,
    Core,
}

pub struct Eac3Decoder {
    buf: Vec<u8>,
    mode: Mode,
    session: Option<OpenJocSession>,
    core: JocAccessUnitPcmDecoder,
    dither: Vec<f64>,
    first_after_reset: bool,
    /// Last decode problem, for diagnostics (read through thd_error).
    pub last_error: Option<String>,
}

impl Default for Eac3Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Eac3Decoder {
    pub fn new() -> Self {
        // Deterministic dither sequence, as used by the OpenJOC session.
        let mut state = 0x6d2b_79f5_u32;
        let dither = (0..32_768)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                (f64::from(state) / f64::from(u32::MAX) - 0.5) * 0.5
            })
            .collect();
        Eac3Decoder {
            buf: Vec::new(),
            mode: Mode::Unknown,
            session: None,
            core: JocAccessUnitPcmDecoder::new(),
            dither,
            first_after_reset: true,
            last_error: None,
        }
    }

    /// Seek or flush: forget buffered data and decoder history.
    pub fn reset(&mut self) {
        self.buf.clear();
        self.session = None;
        self.core.reset();
        self.first_after_reset = true;
    }

    /// Is the current stream rendered from Atmos objects?
    pub fn is_atmos(&self) -> bool {
        self.mode == Mode::Joc
    }

    /// Pushes E-AC-3 bytes; decoded blocks are appended to `out`.
    pub fn push(&mut self, data: &[u8], out: &mut Vec<Block>) {
        self.buf.extend_from_slice(data);
        loop {
            // Resynchronize on the E-AC-3 syncword.
            match self.buf.windows(2).position(|w| w == [0x0B, 0x77]) {
                Some(0) => {}
                Some(p) => {
                    self.buf.drain(..p);
                }
                None => {
                    let keep = self.buf.len().min(1);
                    self.buf.drain(..self.buf.len() - keep);
                    return;
                }
            }
            match parse_access_unit_bounds(&self.buf, false) {
                Ok(AccessUnitParse::NeedMore) => {
                    if self.buf.len() > MAX_BUFFER {
                        self.buf.drain(..2);
                        continue;
                    }
                    return;
                }
                Ok(AccessUnitParse::Complete(len)) if len > 0 && len <= self.buf.len() => {
                    let au: Vec<u8> = self.buf.drain(..len).collect();
                    self.decode_access_unit(&au, out);
                }
                _ => {
                    // Not a 48 kHz General access unit (other rates, damaged data): fall back
                    // to one independent syncframe and its dependents, for the core decoder.
                    match self.plain_unit_len() {
                        Some(0) => return, // needs more data
                        Some(len) => {
                            let au: Vec<u8> = self.buf.drain(..len).collect();
                            self.decode_core(&au, out);
                        }
                        None => {
                            self.buf.drain(..2); // not a syncframe: skip this syncword
                        }
                    }
                }
            }
        }
    }

    /// Length of an independent syncframe plus its dependent syncframes at the start of
    /// the buffer; Some(0) if more data is needed, None if this is not a syncframe.
    fn plain_unit_len(&self) -> Option<usize> {
        let first = parse_syncframe_header(&self.buf).ok()?;
        if first.stream_type == StreamType::Dependent || first.frame_size == 0 {
            return None;
        }
        let mut len = first.frame_size;
        loop {
            if self.buf.len() < len + 8 {
                return Some(0);
            }
            match parse_syncframe_header(&self.buf[len..]) {
                Ok(h) if h.stream_type == StreamType::Dependent && h.frame_size > 0 => len += h.frame_size,
                _ => return Some(len),
            }
        }
    }

    fn decode_access_unit(&mut self, au: &[u8], out: &mut Vec<Block>) {
        if self.mode != Mode::Core {
            if self.session.is_none() {
                let config = OpenJocConfig {
                    speaker_layout: RENDER_LAYOUT.to_owned(),
                    object_output: true,
                    ..OpenJocConfig::default()
                };
                self.session = OpenJocSession::new(config).ok();
            }
            if let Some(session) = self.session.as_mut() {
                let packet = OpenJocPacket {
                    data: au,
                    pts_samples: None,
                    discontinuity: self.first_after_reset,
                    preroll: false,
                };
                match session.push_packet(packet) {
                    Ok(_) => {
                        self.mode = Mode::Joc;
                        self.first_after_reset = false;
                        while let Some(f) = session.receive_frame() {
                            let channels = f.channel_count.min(MAX_CH);
                            let mut elements = [ThdElement::default(); MAX_CH];
                            let mut changes = Vec::new();
                            if let Some(tracks) = &f.object_tracks {
                                // Element state at each update offset within the frame.
                                let mut offsets: Vec<usize> = tracks
                                    .iter()
                                    .flat_map(|t| t.states.iter().map(|st| st.sample_offset))
                                    .collect();
                                offsets.push(0);
                                offsets.sort_unstable();
                                offsets.dedup();
                                for &offset in &offsets {
                                    let mut at = [ThdElement::default(); MAX_CH];
                                    for (c, track) in tracks.iter().take(channels).enumerate() {
                                        at[c] = track_element(track, offset);
                                    }
                                    if offset == 0 {
                                        elements = at;
                                    } else {
                                        changes.push((offset, at));
                                    }
                                }
                            } else {
                                for (c, label) in f.channel_labels.iter().take(channels).enumerate() {
                                    elements[c] = label_element(label);
                                }
                            }
                            let pcm = if channels == f.channel_count {
                                f.interleaved_f32
                            } else {
                                f.interleaved_f32
                                    .chunks_exact(f.channel_count)
                                    .flat_map(|s| s[..channels].to_vec())
                                    .collect()
                            };
                            out.push(Block {
                                pcm,
                                channels,
                                samples: f.sample_count,
                                sample_rate: f.sample_rate,
                                elements,
                                changes,
                            });
                        }
                        return;
                    }
                    Err(e) => {
                        // A session stops at its first error: a new one is created for the next unit.
                        self.session = None;
                        if self.mode == Mode::Unknown {
                            self.last_error = Some(format!("OpenJOC push_packet: {e:?}"));
                        }
                        if self.mode == Mode::Unknown && matches!(e, OpenJocError::InvalidPacket(_)) {
                            self.mode = Mode::Core; // no Atmos in this stream
                        }
                        // This unit is decoded by the core decoder (no gap).
                    }
                }
            }
        }
        self.decode_core(au, out);
    }

    fn decode_core(&mut self, au: &[u8], out: &mut Vec<Block>) {
        let frames = match index_syncframes(au) {
            Ok(f) => f,
            Err(e) => {
                self.last_error = Some(format!("core: index_syncframes: {e:?}"));
                return;
            }
        };
        let units = match group_access_units(&frames) {
            Ok(u) => u,
            Err(e) => {
                self.last_error = Some(format!("core: group_access_units: {e:?}"));
                return;
            }
        };
        for unit in units {
            let pcm = match self.core.decode(au, &frames, unit, &self.dither) {
                Ok(p) => p,
                Err(e) => {
                    self.last_error = Some(format!("core: decode: {e:?}"));
                    self.core.reset();
                    continue;
                }
            };
            let n = pcm.samples as usize;
            let mut channels: Vec<(&Vec<f64>, ThdElement)> = pcm
                .channel_locations
                .iter()
                .zip(pcm.channels.iter())
                .map(|(l, data)| (data, location_element(*l)))
                .collect();
            if let Some(lfe) = &pcm.lfe {
                channels.push((lfe, ThdElement { kind: KIND_LFE, active: 1, gain: 1.0, ..Default::default() }));
            }
            channels.truncate(MAX_CH);
            let mut elements = [ThdElement::default(); MAX_CH];
            for (c, (_, e)) in channels.iter().enumerate() {
                elements[c] = *e;
            }
            let mut data = Vec::with_capacity(n * channels.len());
            for s in 0..n {
                for (ch, _) in &channels {
                    data.push(ch.get(s).copied().unwrap_or(0.0) as f32);
                }
            }
            out.push(Block {
                pcm: data,
                channels: channels.len(),
                samples: n,
                sample_rate: pcm.sample_rate,
                elements,
                changes: Vec::new(),
            });
        }
    }
}

/// Element of an OpenJOC object track at a sample offset of its frame.
fn track_element(track: &openjoc_api::OpenJocObjectTrack, offset: usize) -> ThdElement {
    if track.lfe {
        return ThdElement { kind: KIND_LFE, active: 1, gain: 1.0, ..Default::default() };
    }
    let Some(state) = track.states.iter().rev().find(|st| st.sample_offset <= offset).or(track.states.first())
    else {
        return ThdElement::default();
    };
    let mut e = if state.coordinates.len() >= 3 {
        // OpenJOC room coordinates (x left->right, y front->back, z floor->ceiling, 0..1)
        // to DAMF (x left/right -1..1, y back/front -1..1, z floor/ceiling 0..1).
        let c = &state.coordinates;
        object(
            (2.0 * c[0] - 1.0).clamp(-1.0, 1.0) as f32,
            (1.0 - 2.0 * c[1]).clamp(-1.0, 1.0) as f32,
            c[2].clamp(0.0, 1.0) as f32,
        )
    } else {
        label_element(&state.identity)
    };
    if e.kind == KIND_OBJECT {
        e.gain = state.gain.max(0.0) as f32;
        e.active = u32::from(state.active && e.active != 0);
    }
    e
}

fn object(x: f32, y: f32, z: f32) -> ThdElement {
    ThdElement { kind: KIND_OBJECT, active: 1, x, y, z, gain: 1.0 }
}

/// Fixed position of an OpenJOC output channel (DAMF: x left/right, y back/front, z floor/ceiling).
fn label_element(label: &str) -> ThdElement {
    match label {
        "LFE" | "LFE1" | "LFE2" => ThdElement { kind: KIND_LFE, active: 1, gain: 1.0, ..Default::default() },
        "FL" | "L" => object(-1.0, 1.0, 0.0),
        "FR" | "R" => object(1.0, 1.0, 0.0),
        "FC" | "C" => object(0.0, 1.0, 0.0),
        "Ls" | "SL" => object(-1.0, 0.0, 0.0),
        "Rs" | "SR" => object(1.0, 0.0, 0.0),
        "Lb" | "BL" | "Lrs" => object(-1.0, -1.0, 0.0),
        "Rb" | "BR" | "Rrs" => object(1.0, -1.0, 0.0),
        "Cb" | "BC" | "Cs" => object(0.0, -1.0, 0.0),
        "Lw" => object(-1.0, 0.68, 0.0),
        "Rw" => object(1.0, 0.68, 0.0),
        "Ltf" | "TFL" | "Tfl" => object(-1.0, 1.0, 1.0),
        "Rtf" | "TFR" | "Tfr" => object(1.0, 1.0, 1.0),
        "Ltm" | "TSL" | "Tsl" => object(-1.0, 0.0, 1.0),
        "Rtm" | "TSR" | "Tsr" => object(1.0, 0.0, 1.0),
        "Ltr" | "TBL" | "Tbl" | "Ltb" => object(-1.0, -1.0, 1.0),
        "Rtr" | "TBR" | "Tbr" | "Rtb" => object(1.0, -1.0, 1.0),
        "Tc" | "TC" => object(0.0, 0.0, 1.0),
        _ => ThdElement::default(), // unknown: muted
    }
}

fn location_element(l: ChannelLocation) -> ThdElement {
    match l {
        ChannelLocation::Left => object(-1.0, 1.0, 0.0),
        ChannelLocation::Right => object(1.0, 1.0, 0.0),
        ChannelLocation::Centre => object(0.0, 1.0, 0.0),
        ChannelLocation::LeftSurround => object(-1.0, 0.0, 0.0),
        ChannelLocation::RightSurround => object(1.0, 0.0, 0.0),
        ChannelLocation::LeftBack => object(-1.0, -1.0, 0.0),
        ChannelLocation::RightBack => object(1.0, -1.0, 0.0),
        ChannelLocation::TopFrontLeft => object(-1.0, 1.0, 1.0),
        ChannelLocation::TopFrontRight => object(1.0, 1.0, 1.0),
        ChannelLocation::Lfe(_) => ThdElement { kind: KIND_LFE, active: 1, gain: 1.0, ..Default::default() },
        ChannelLocation::Other(_) => ThdElement::default(),
    }
}
