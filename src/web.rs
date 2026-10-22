//! Wi-Fi SoftAP + HTTP control server on Core 0.
//!
//! Brings up open SoftAP `rinsynth-esp32` (192.168.71.1) and serves the
//! single-page synth controller UI (`index.html`), the 85-preset factory
//! bank list (`/presets`), preset loader (`/preset?i=N`), and real-time
//! parameter/mod-matrix/note endpoints (`/cmd?k=...`).
//!
//! All HTTP handling runs on Core 0 and communicates with the Core 1 audio
//! render loop exclusively through lock-free SPSC rings.

use core::cell::UnsafeCell;
use core::ffi::CStr;
use core::sync::atomic::{AtomicUsize, Ordering};
use esp_idf_hal::modem::Modem;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{
    AccessPointConfiguration, AuthMethod, Configuration as WifiConfig, EspWifi,
};
use esp_idf_sys::*;
use log::info;
use rinsynth_dsp::embedded::EmbeddedEngine;
use rinsynth_dsp::FilterMode;
use rinsynth_model::event::SynthEvent;
use rinsynth_model::lfo::LfoWaveform;
use rinsynth_model::mod_matrix::{ModDest, ModSlot, ModSource};
use rinsynth_model::params::SynthParam;
use rinsynth_model::presets::Preset;
use std::sync::OnceLock;

use crate::usb_midi;

const AP_SSID: &str = "rinsynth-esp32";
const INDEX_HTML: &str = include_str!("index.html");
const CMD_RING_CAP: usize = 64;

#[derive(Clone, Copy)]
pub enum WebCmd {
    Nop,
    Param(SynthParam, f32),
    FilterMode(u8),
    ModWheel(f32),
    Lfo2Rate(f32),
    Lfo2Depth(f32),
    Lfo2Wf(u8),
    Env3Attack(f32),
    Env3Decay(f32),
    Env3Sustain(f32),
    Env3Release(f32),
    ModSlot(u8, ModSlot),
    ChorusRate(f32),
    ChorusDepth(f32),
    ChorusMix(f32),
    DelayTime(f32),
    DelayFb(f32),
    DelayMix(f32),
    RevRoom(f32),
    RevDamp(f32),
    RevMix(f32),
}

struct CmdRing {
    buf: UnsafeCell<[WebCmd; CMD_RING_CAP]>,
    head: AtomicUsize,
    tail: AtomicUsize,
}

unsafe impl Sync for CmdRing {}

impl CmdRing {
    const fn new() -> Self {
        Self {
            buf: UnsafeCell::new([WebCmd::Nop; CMD_RING_CAP]),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    fn push(&self, cmd: WebCmd) {
        let h = self.head.load(Ordering::Relaxed);
        let n = (h + 1) & (CMD_RING_CAP - 1);
        if n == self.tail.load(Ordering::Acquire) {
            return;
        }
        unsafe { (*self.buf.get())[h] = cmd };
        self.head.store(n, Ordering::Release);
    }

    fn pop(&self) -> Option<WebCmd> {
        let t = self.tail.load(Ordering::Relaxed);
        if t == self.head.load(Ordering::Acquire) {
            return None;
        }
        let c = unsafe { (*self.buf.get())[t] };
        self.tail
            .store((t + 1) & (CMD_RING_CAP - 1), Ordering::Release);
        Some(c)
    }
}

static CMD_RING: CmdRing = CmdRing::new();
static PRESETS_LIST_JSON: OnceLock<String> = OnceLock::new();
static PRESET_DETAILS_JSON: OnceLock<Vec<String>> = OnceLock::new();

pub fn pop_cmd() -> Option<WebCmd> {
    CMD_RING.pop()
}

const MOD_SOURCES: [ModSource; 14] = [
    ModSource::Off,
    ModSource::Lfo1,
    ModSource::Lfo2,
    ModSource::AmpEnv,
    ModSource::FilterEnv,
    ModSource::Env3,
    ModSource::Velocity,
    ModSource::KeyTrack,
    ModSource::ModWheel,
    ModSource::Aftertouch,
    ModSource::PitchBend,
    ModSource::Constant,
    ModSource::Lfo3,
    ModSource::Env4,
];

const MOD_DESTS: [ModDest; 18] = [
    ModDest::Off,
    ModDest::AllPitch,
    ModDest::FilterCutoff,
    ModDest::FilterResonance,
    ModDest::Amplitude,
    ModDest::Osc1Pitch,
    ModDest::Osc2Pitch,
    ModDest::Osc3Pitch,
    ModDest::Osc1Level,
    ModDest::Osc2Level,
    ModDest::Osc3Level,
    ModDest::Osc1Pan,
    ModDest::Osc2Pan,
    ModDest::Osc3Pan,
    ModDest::Osc1Pw,
    ModDest::Osc2Pw,
    ModDest::Osc3Pw,
    ModDest::Lfo1Rate,
];

fn src_to_idx(s: ModSource) -> usize {
    MOD_SOURCES.iter().position(|&x| x == s).unwrap_or(0)
}

fn dst_to_idx(d: ModDest) -> usize {
    MOD_DESTS.iter().position(|&x| x == d).unwrap_or(0)
}

pub fn apply_cmd(vm: &mut EmbeddedEngine, cmd: WebCmd) {
    match cmd {
        WebCmd::Nop => {}
        WebCmd::Param(p, v) => vm.set_param(p, v),
        WebCmd::FilterMode(m) => {
            let mode = match m {
                1 => FilterMode::HighPass,
                2 => FilterMode::BandPass,
                3 => FilterMode::Notch,
                _ => FilterMode::LowPass,
            };
            vm.set_filter_mode(mode);
        }
        WebCmd::ModWheel(v) => vm.set_mod_wheel(v),
        WebCmd::Lfo2Rate(v) => vm.set_lfo2_rate(v),
        WebCmd::Lfo2Depth(v) => vm.set_lfo2_depth(v),
        WebCmd::Lfo2Wf(w) => {
            let wf = match w {
                0 => LfoWaveform::Sine,
                1 => LfoWaveform::Triangle,
                2 => LfoWaveform::Square,
                3 => LfoWaveform::Saw,
                _ => LfoWaveform::Random,
            };
            vm.set_lfo2_waveform(wf);
        }
        WebCmd::Env3Attack(v) => vm.set_env3_attack(v),
        WebCmd::Env3Decay(v) => vm.set_env3_decay(v),
        WebCmd::Env3Sustain(v) => vm.set_env3_sustain(v),
        WebCmd::Env3Release(v) => vm.set_env3_release(v),
        WebCmd::ModSlot(idx, slot) => vm.set_mod_slot(idx as usize, slot),
        WebCmd::ChorusRate(v) => vm.effects.set_chorus_rate(v),
        WebCmd::ChorusDepth(v) => vm.effects.set_chorus_depth(v),
        WebCmd::ChorusMix(v) => {
            vm.effects.set_chorus_enabled(v > 0.001);
            vm.effects.set_chorus_mix(v);
        }
        WebCmd::DelayTime(v) => vm.effects.set_delay_time(v),
        WebCmd::DelayFb(v) => vm.effects.set_delay_feedback(v),
        WebCmd::DelayMix(v) => {
            vm.effects.set_delay_enabled(v > 0.001);
            vm.effects.set_delay_mix(v);
        }
        WebCmd::RevRoom(v) => vm.effects.set_reverb_room_size(v),
        WebCmd::RevDamp(v) => vm.effects.set_reverb_damping(v),
        WebCmd::RevMix(v) => {
            vm.effects.set_reverb_enabled(v > 0.001);
            vm.effects.set_reverb_mix(v);
        }
    }
}

fn preset_to_ui_json(p: &Preset) -> String {
    let mut m_json = String::with_capacity(256);
    m_json.push('[');
    for i in 0..8 {
        if i > 0 {
            m_json.push(',');
        }
        let s = &p.mod_matrix.slots[i];
        m_json.push_str(&format!(
            "[{},{},{},{:.2}]",
            if s.enabled { 1 } else { 0 },
            src_to_idx(s.source),
            dst_to_idx(s.dest),
            s.amount
        ));
    }
    m_json.push(']');

    format!(
        concat!(
            "{{",
            "\"o1w\":{},\"o2w\":{},\"o3w\":{},",
            "\"o1l\":{:.3},\"o2l\":{:.3},\"o3l\":{:.3},",
            "\"o1pw\":{:.3},\"o2pw\":{:.3},\"o3pw\":{:.3},",
            "\"o2s\":{},\"o2c\":{},\"o3s\":{},\"o3c\":{},",
            "\"o1pn\":{:.3},\"o2pn\":{:.3},\"o3pn\":{:.3},",
            "\"cut\":{:.1},\"res\":{:.3},\"fenv\":{:.3},",
            "\"fatk\":{:.3},\"fdec\":{:.3},\"fsus\":{:.3},\"frel\":{:.3},",
            "\"wth\":{:.3},",
            "\"atk\":{:.3},\"dec\":{:.3},\"sus\":{:.3},\"rel\":{:.3},\"prt\":{:.3},",
            "\"e3a\":{:.3},\"e3d\":{:.3},\"e3s\":{:.3},\"e3r\":{:.3},",
            "\"ldst\":{},\"lwf\":{},\"lrt\":{:.3},\"ldep\":{:.3},",
            "\"l2wf\":{},\"l2rt\":{:.3},\"l2dep\":{:.3},",
            "\"ch_r\":{:.3},\"ch_d\":{:.3},\"ch_m\":{:.3},",
            "\"dl_t\":{:.3},\"dl_fb\":{:.3},\"dl_m\":{:.3},",
            "\"rv_r\":{:.3},\"rv_d\":{:.3},\"rv_m\":{:.3},",
            "\"vol\":{:.3},\"m\":{}",
            "}}"
        ),
        p.osc1_waveform as u8,
        p.osc2_waveform as u8,
        p.osc3_waveform as u8,
        p.osc1_level,
        p.osc2_level,
        p.osc3_level,
        p.osc1_pw,
        p.osc2_pw,
        p.osc3_pw,
        p.osc2_semitones,
        p.osc2_cents,
        p.osc3_semitones,
        p.osc3_cents,
        p.osc1_pan,
        p.osc2_pan,
        p.osc3_pan,
        p.filter_cutoff,
        p.filter_resonance,
        p.filter_env_amount,
        p.filter_attack,
        p.filter_decay,
        p.filter_sustain,
        p.filter_release,
        p.stereo_width,
        p.amp_attack,
        p.amp_decay,
        p.amp_sustain,
        p.amp_release,
        p.portamento_time,
        p.env3_attack,
        p.env3_decay,
        p.env3_sustain,
        p.env3_release,
        p.lfo_destination as u8,
        p.lfo_waveform as u8,
        p.lfo_rate,
        p.lfo_depth,
        p.lfo2_waveform as u8,
        p.lfo2_rate,
        p.lfo2_depth,
        p.chorus_rate,
        p.chorus_depth,
        if p.chorus_enabled { p.chorus_mix } else { 0.0 },
        p.delay_time,
        p.delay_feedback,
        if p.delay_enabled { p.delay_mix } else { 0.0 },
        p.reverb_room_size,
        p.reverb_damping,
        if p.reverb_enabled { p.reverb_mix } else { 0.0 },
        p.master_volume,
        m_json,
    )
}

fn handle_query(q: &str) {
    let mut key = "";
    let mut val = "";
    let mut slot_idx = 0u8;
    let mut slot_en = false;
    let mut slot_src = 0usize;
    let mut slot_dst = 0usize;
    let mut slot_amt = 0.0f32;
    for pair in q.split('&') {
        if let Some((k, v)) = pair.split_once('=') {
            match k {
                "k" => key = v,
                "v" => val = v,
                "s" => slot_idx = v.parse().unwrap_or(0),
                "en" => slot_en = v == "1",
                "src" => slot_src = v.parse().unwrap_or(0),
                "dst" => slot_dst = v.parse().unwrap_or(0),
                "amt" => slot_amt = v.parse().unwrap_or(0.0),
                _ => {}
            }
        }
    }
    if key == "mslot" {
        let slot = ModSlot {
            enabled: slot_en,
            source: *MOD_SOURCES.get(slot_src).unwrap_or(&ModSource::Off),
            dest: *MOD_DESTS.get(slot_dst).unwrap_or(&ModDest::Off),
            amount: slot_amt.clamp(-1.0, 1.0),
        };
        CMD_RING.push(WebCmd::ModSlot(slot_idx, slot));
        return;
    }
    let Ok(v) = val.parse::<f32>() else { return };
    match key {
        "on" => usb_midi::push_event(SynthEvent::note_on_midi(v as u8, 108)),
        "off" => usb_midi::push_event(SynthEvent::note_off(v as u8)),
        "o1w" => CMD_RING.push(WebCmd::Param(SynthParam::Osc1Waveform, v)),
        "o2w" => CMD_RING.push(WebCmd::Param(SynthParam::Osc2Waveform, v)),
        "o3w" => CMD_RING.push(WebCmd::Param(SynthParam::Osc3Waveform, v)),
        "o1l" => CMD_RING.push(WebCmd::Param(SynthParam::Osc1Level, v)),
        "o2l" => CMD_RING.push(WebCmd::Param(SynthParam::Osc2Level, v)),
        "o3l" => CMD_RING.push(WebCmd::Param(SynthParam::Osc3Level, v)),
        "o1pw" => CMD_RING.push(WebCmd::Param(SynthParam::Osc1Pw, v)),
        "o2pw" => CMD_RING.push(WebCmd::Param(SynthParam::Osc2Pw, v)),
        "o3pw" => CMD_RING.push(WebCmd::Param(SynthParam::Osc3Pw, v)),
        "o2s" => CMD_RING.push(WebCmd::Param(SynthParam::Osc2Semitones, v)),
        "o2c" => CMD_RING.push(WebCmd::Param(SynthParam::Osc2Cents, v)),
        "o3s" => CMD_RING.push(WebCmd::Param(SynthParam::Osc3Semitones, v)),
        "o3c" => CMD_RING.push(WebCmd::Param(SynthParam::Osc3Cents, v)),
        "o1pn" => CMD_RING.push(WebCmd::Param(SynthParam::Osc1Pan, v)),
        "o2pn" => CMD_RING.push(WebCmd::Param(SynthParam::Osc2Pan, v)),
        "o3pn" => CMD_RING.push(WebCmd::Param(SynthParam::Osc3Pan, v)),
        "fmode" => CMD_RING.push(WebCmd::FilterMode(v as u8)),
        "cut" => CMD_RING.push(WebCmd::Param(SynthParam::FilterCutoff, v)),
        "res" => CMD_RING.push(WebCmd::Param(SynthParam::FilterResonance, v)),
        "fenv" => CMD_RING.push(WebCmd::Param(SynthParam::FilterEnvAmount, v)),
        "fatk" => CMD_RING.push(WebCmd::Param(SynthParam::FilterAttack, v)),
        "fdec" => CMD_RING.push(WebCmd::Param(SynthParam::FilterDecay, v)),
        "fsus" => CMD_RING.push(WebCmd::Param(SynthParam::FilterSustain, v)),
        "frel" => CMD_RING.push(WebCmd::Param(SynthParam::FilterRelease, v)),
        "wth" => CMD_RING.push(WebCmd::Param(SynthParam::StereoWidth, v)),
        "atk" => CMD_RING.push(WebCmd::Param(SynthParam::Attack, v)),
        "dec" => CMD_RING.push(WebCmd::Param(SynthParam::Decay, v)),
        "sus" => CMD_RING.push(WebCmd::Param(SynthParam::Sustain, v)),
        "rel" => CMD_RING.push(WebCmd::Param(SynthParam::Release, v)),
        "prt" => CMD_RING.push(WebCmd::Param(SynthParam::PortamentoTime, v)),
        "e3a" => CMD_RING.push(WebCmd::Env3Attack(v)),
        "e3d" => CMD_RING.push(WebCmd::Env3Decay(v)),
        "e3s" => CMD_RING.push(WebCmd::Env3Sustain(v)),
        "e3r" => CMD_RING.push(WebCmd::Env3Release(v)),
        "mw" => CMD_RING.push(WebCmd::ModWheel(v)),
        "ldst" => CMD_RING.push(WebCmd::Param(SynthParam::LfoDestination, v)),
        "lwf" => CMD_RING.push(WebCmd::Param(SynthParam::LfoWaveform, v)),
        "lrt" => CMD_RING.push(WebCmd::Param(SynthParam::LfoRate, v)),
        "ldep" => CMD_RING.push(WebCmd::Param(SynthParam::LfoDepth, v)),
        "l2wf" => CMD_RING.push(WebCmd::Lfo2Wf(v as u8)),
        "l2rt" => CMD_RING.push(WebCmd::Lfo2Rate(v)),
        "l2dep" => CMD_RING.push(WebCmd::Lfo2Depth(v)),
        "ch_r" => CMD_RING.push(WebCmd::ChorusRate(v)),
        "ch_d" => CMD_RING.push(WebCmd::ChorusDepth(v)),
        "ch_m" => CMD_RING.push(WebCmd::ChorusMix(v)),
        "dl_t" => CMD_RING.push(WebCmd::DelayTime(v)),
        "dl_fb" => CMD_RING.push(WebCmd::DelayFb(v)),
        "dl_m" => CMD_RING.push(WebCmd::DelayMix(v)),
        "rv_r" => CMD_RING.push(WebCmd::RevRoom(v)),
        "rv_d" => CMD_RING.push(WebCmd::RevDamp(v)),
        "rv_m" => CMD_RING.push(WebCmd::RevMix(v)),
        "vol" => CMD_RING.push(WebCmd::Param(SynthParam::MasterVolume, v)),
        _ => {}
    }
}

unsafe extern "C" fn index_get(req: *mut httpd_req_t) -> esp_err_t {
    httpd_resp_set_type(req, c"text/html; charset=utf-8".as_ptr());
    httpd_resp_send(req, INDEX_HTML.as_ptr() as *const _, INDEX_HTML.len() as isize)
}

unsafe extern "C" fn presets_get(req: *mut httpd_req_t) -> esp_err_t {
    let body = PRESETS_LIST_JSON.get().map(|s| s.as_str()).unwrap_or("[]");
    httpd_resp_set_type(req, c"application/json".as_ptr());
    httpd_resp_send(req, body.as_ptr() as *const _, body.len() as isize)
}

unsafe extern "C" fn preset_get(req: *mut httpd_req_t) -> esp_err_t {
    let uri = CStr::from_ptr((*req).uri.as_ptr()).to_str().unwrap_or("");
    let idx = uri
        .split_once("i=")
        .and_then(|(_, s)| s.split('&').next())
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(0);
    usb_midi::set_program_change(idx as u8);
    let body = PRESET_DETAILS_JSON
        .get()
        .and_then(|v| v.get(idx))
        .map(|s| s.as_str())
        .unwrap_or("{}");
    httpd_resp_set_type(req, c"application/json".as_ptr());
    httpd_resp_send(req, body.as_ptr() as *const _, body.len() as isize)
}

unsafe extern "C" fn cmd_get(req: *mut httpd_req_t) -> esp_err_t {
    let uri = CStr::from_ptr((*req).uri.as_ptr()).to_str().unwrap_or("");
    if let Some((_, q)) = uri.split_once('?') {
        handle_query(q);
    }
    httpd_resp_send(req, c"ok".as_ptr(), 2)
}

unsafe extern "C" fn ws_get(req: *mut httpd_req_t) -> esp_err_t {
    if (*req).method == http_method_HTTP_GET as i32 {
        return ESP_OK; // completes the WebSocket handshake
    }
    let mut pkt: httpd_ws_frame_t = core::mem::zeroed();
    pkt.type_ = httpd_ws_type_t_HTTPD_WS_TYPE_TEXT;

    // Step 1: fetch frame header & payload length. Return `ret` on any error
    // so `httpd` closes dead/invalid sockets instead of livelocking!
    let ret = httpd_ws_recv_frame(req, &mut pkt, 0);
    if ret != ESP_OK {
        return ret;
    }
    if pkt.len == 0 || pkt.len > 255 {
        return ESP_OK;
    }

    let mut buf = [0u8; 256];
    pkt.payload = buf.as_mut_ptr();
    let ret = httpd_ws_recv_frame(req, &mut pkt, pkt.len);
    if ret != ESP_OK {
        return ret;
    }

    if pkt.type_ == httpd_ws_type_t_HTTPD_WS_TYPE_TEXT {
        if let Ok(msg) = core::str::from_utf8(&buf[..pkt.len]) {
            if let Some(rest) = msg.strip_prefix("preset:") {
                let idx = rest.trim().parse::<usize>().unwrap_or(0);
                usb_midi::set_program_change(idx as u8);
                if let Some(body) = PRESET_DETAILS_JSON.get().and_then(|v| v.get(idx)) {
                    let mut out: httpd_ws_frame_t = core::mem::zeroed();
                    out.final_ = true;
                    out.fragmented = false;
                    out.type_ = httpd_ws_type_t_HTTPD_WS_TYPE_TEXT;
                    out.payload = body.as_ptr() as *mut u8;
                    out.len = body.len();
                    let _ = httpd_ws_send_frame(req, &mut out);
                }
            } else {
                handle_query(msg);
            }
        }
    }
    ESP_OK
}

/// Start the SoftAP + HTTP/WS server on Core 0.
pub fn start(modem: Modem<'static>, bank: &[Preset]) -> anyhow::Result<()> {
    // Pre-build preset directory & per-preset UI JSON in PSRAM
    let mut list_json = String::with_capacity(4096);
    list_json.push('[');
    let mut details = Vec::with_capacity(bank.len());
    for (i, p) in bank.iter().enumerate() {
        if i > 0 {
            list_json.push(',');
        }
        list_json.push_str(&format!(
            "{{\"i\":{},\"n\":\"{}\",\"c\":\"{}\"}}",
            i, p.name, p.category
        ));
        details.push(preset_to_ui_json(p));
    }
    list_json.push(']');
    let _ = PRESETS_LIST_JSON.set(list_json);
    let _ = PRESET_DETAILS_JSON.set(details);

    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let mut wifi = Box::new(EspWifi::new(modem, sysloop, Some(nvs))?);
    wifi.set_configuration(&WifiConfig::AccessPoint(AccessPointConfiguration {
        ssid: AP_SSID.try_into().unwrap(),
        channel: 1,
        auth_method: AuthMethod::None,
        max_connections: 4,
        ..Default::default()
    }))?;
    wifi.start()?;
    unsafe {
        esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE);

        let cfg = httpd_config_t {
            task_priority: 5,
            stack_size: 8192,
            core_id: 0,
            task_caps: MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
            server_port: 80,
            ctrl_port: 32768,
            max_open_sockets: 7,
            max_uri_handlers: 8,
            max_resp_headers: 8,
            backlog_conn: 5,
            lru_purge_enable: true,
            recv_wait_timeout: 5,
            send_wait_timeout: 5,
            ..Default::default()
        };
        let mut srv: httpd_handle_t = core::ptr::null_mut();
        esp!(httpd_start(&mut srv, &cfg))?;

        let uris = [
            httpd_uri_t {
                uri: c"/".as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(index_get),
                ..Default::default()
            },
            httpd_uri_t {
                uri: c"/presets".as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(presets_get),
                ..Default::default()
            },
            httpd_uri_t {
                uri: c"/preset".as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(preset_get),
                ..Default::default()
            },
            httpd_uri_t {
                uri: c"/cmd".as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(cmd_get),
                ..Default::default()
            },
            httpd_uri_t {
                uri: c"/ws".as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(ws_get),
                is_websocket: true,
                ..Default::default()
            },
        ];
        for u in &uris {
            esp!(httpd_register_uri_handler(srv, u))?;
        }
    }
    Box::leak(wifi);

    info!("web: SoftAP '{}' ready — browse to http://192.168.71.1", AP_SSID);
    Ok(())
}
