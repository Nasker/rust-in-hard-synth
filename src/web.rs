//! Wi-Fi SoftAP + STA (Mixed mode) + Captive Portal DNS + HTTP/WS server on Core 0.
//!
//! - Always brings up open SoftAP `rinsynth-esp32` (`192.168.71.1`).
//! - Runs a UDP port 53 DNS responder + HTTP 302 captive-portal handler so
//!   phones/tablets pop open the synth UI automatically and keep the Wi-Fi
//!   route pinned instead of falling back to mobile data.
//! - Optionally connects as a Wi-Fi Station (AP+STA `Mixed` mode) when an
//!   SSID/password is configured via the Web UI (persisted in NVS `"wifi"`).
//! - Streams real-time synth parameters, 4 oscillators, 8-slot ModMatrix,
//!   and preset changes over `/ws` (with `/cmd` HTTP fallback).

use std::ffi::{CStr, CString};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use esp_idf_hal::modem::Modem;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{
    AccessPointConfiguration, AuthMethod, ClientConfiguration, Configuration as WifiConfig,
    EspWifi,
};
use esp_idf_sys::*;
use log::info;
use rinsynth_model::event::SynthEvent;
use rinsynth_model::mod_matrix::{ModDest, ModSlot, ModSource};
use rinsynth_model::params::SynthParam;
use rinsynth_model::presets::Preset;
use rinsynth_model::voice::PolyphonyMode;
use std::net::UdpSocket;
use std::sync::{Mutex, OnceLock};

use rinsynth_esp32::control::{self, push_cmd, SynthCommand};

const AP_SSID: &str = "rinsynth-esp32";
const AP_IP: [u8; 4] = [192, 168, 71, 1];
const INDEX_HTML: &str = include_str!("index.html");
/// Raw httpd handle stored as usize so a background thread can queue WS work.
static HTTPD_HANDLE: AtomicUsize = AtomicUsize::new(0);
/// Must match `max_open_sockets` in the httpd config below.
const MAX_HTTPD_SESSIONS: usize = 12;
static PRESETS_LIST_JSON: OnceLock<String> = OnceLock::new();
static PRESET_DETAILS_JSON: OnceLock<Vec<String>> = OnceLock::new();
static WIFI: Mutex<Option<EspWifi<'static>>> = Mutex::new(None);
static SAVED_SSID: Mutex<String> = Mutex::new(String::new());

/// Wi-Fi radio on/off state for the rotary double-click RF-EMI A/B test.
/// Independent of the WS `wstop` bounce — this one STAYS off until the
/// user toggles it back (no timer), so the ~10 Hz beacon clicks are easy
/// to compare. Stopping the radio kills the web UI until re-enabled.
static WIFI_RADIO_ON: AtomicBool = AtomicBool::new(true);

pub fn wifi_radio_on() -> bool {
    WIFI_RADIO_ON.load(Ordering::Relaxed)
}

/// Toggle the Wi-Fi radio. Returns the new state. The actual
/// `esp_wifi_stop/start` runs on a helper thread so the caller (UI task)
/// doesn't stall the OLED on the driver's internal waits.
pub fn toggle_wifi_radio() -> bool {
    let on = !wifi_radio_on();
    WIFI_RADIO_ON.store(on, Ordering::Relaxed);
    let _ = std::thread::Builder::new()
        .stack_size(3072)
        .spawn(move || unsafe {
            if on {
                esp_wifi_start();
                esp_wifi_connect();
            } else {
                esp_wifi_stop();
            }
        });
    on
}

const MOD_SOURCES: [ModSource; 15] = [
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
    ModSource::Osc4,
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
            "\"o1w\":{},\"o2w\":{},\"o3w\":{},\"o4w\":{},",
            "\"o1l\":{:.3},\"o2l\":{:.3},\"o3l\":{:.3},\"o4l\":{:.3},",
            "\"o1pw\":{:.3},\"o2pw\":{:.3},\"o3pw\":{:.3},\"o4pw\":{:.3},",
            "\"o2s\":{},\"o2c\":{},\"o3s\":{},\"o3c\":{},\"o4s\":{},\"o4c\":{},",
            "\"o1pn\":{:.3},\"o2pn\":{:.3},\"o3pn\":{:.3},\"o4pn\":{:.3},",
            "\"cut\":{:.1},\"res\":{:.3},\"fenv\":{:.3},",
            "\"fatk\":{:.3},\"fdec\":{:.3},\"fsus\":{:.3},\"frel\":{:.3},",
            "\"wth\":{:.3},",
            "\"atk\":{:.3},\"dec\":{:.3},\"sus\":{:.3},\"rel\":{:.3},\"prt\":{:.3},",
            "\"e3a\":{:.3},\"e3d\":{:.3},\"e3s\":{:.3},\"e3r\":{:.3},",
            "\"ldst\":{},\"lwf\":{},\"lrt\":{:.3},\"ldep\":{:.3},",
            "\"l2wf\":{},\"l2rt\":{:.3},\"l2dep\":{:.3},",
            "\"bpm\":{:.1},\"l1_s\":{},\"l1_b\":{:.4},\"l2_s\":{},\"l2_b\":{:.4},\"dl_s\":{},\"dl_b\":{:.4},",
            "\"poly\":{},\"uni\":{},\"udet\":{:.1},\"uspr\":{:.2},",
            "\"dt_e\":{},\"dt_t\":{},\"dt_d\":{:.3},\"dt_m\":{:.3},",
        "\"ch_r\":{:.3},\"ch_d\":{:.3},\"ch_m\":{:.3},",
            "\"dl_t\":{:.3},\"dl_fb\":{:.3},\"dl_m\":{:.3},",
            "\"rv_r\":{:.3},\"rv_d\":{:.3},\"rv_m\":{:.3},",
            "\"vol\":{:.3},\"m\":{}",
            "}}"
        ),
        p.osc1_waveform as u8,
        p.osc2_waveform as u8,
        p.osc3_waveform as u8,
        p.osc4_waveform as u8,
        p.osc1_level,
        p.osc2_level,
        p.osc3_level,
        p.osc4_level,
        p.osc1_pw,
        p.osc2_pw,
        p.osc3_pw,
        p.osc4_pw,
        p.osc2_semitones,
        p.osc2_cents,
        p.osc3_semitones,
        p.osc3_cents,
        p.osc4_semitones,
        p.osc4_cents,
        p.osc1_pan,
        p.osc2_pan,
        p.osc3_pan,
        p.osc4_pan,
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
        p.tempo_bpm,
        if p.lfo1_sync { 1 } else { 0 },
        p.lfo1_beats,
        if p.lfo2_sync { 1 } else { 0 },
        p.lfo2_beats,
        if p.delay_sync { 1 } else { 0 },
        p.delay_beats,
        if p.polyphony_mode == PolyphonyMode::Poly { 1 } else { 0 },
        p.unison_voices,
        p.unison_detune,
        p.unison_spread,
        if p.distortion_enabled { 1 } else { 0 },
        p.distortion_type.index(),
        p.distortion_drive,
        p.distortion_mix,
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

/// Parse one or more `;`-separated query commands (Web UI batches slider
/// updates into a single WS frame to keep message rate bounded).
fn handle_query(q: &str) {
    for seg in q.split(';') {
        if !seg.is_empty() {
            handle_query_one(seg);
        }
    }
}

fn handle_query_one(q: &str) {
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
        push_cmd(SynthCommand::ModSlot(slot_idx, slot));
        return;
    }
    let Ok(v) = val.parse::<f32>() else { return };
    match key {
        "on" => control::push_event(SynthEvent::note_on_midi(v as u8, 108)),
        "off" => control::push_event(SynthEvent::note_off(v as u8)),
        "o1w" => push_cmd(SynthCommand::Param(SynthParam::Osc1Waveform, v)),
        "o2w" => push_cmd(SynthCommand::Param(SynthParam::Osc2Waveform, v)),
        "o3w" => push_cmd(SynthCommand::Param(SynthParam::Osc3Waveform, v)),
        "o4w" => push_cmd(SynthCommand::Param(SynthParam::Osc4Waveform, v)),
        "o1l" => push_cmd(SynthCommand::Param(SynthParam::Osc1Level, v)),
        "o2l" => push_cmd(SynthCommand::Param(SynthParam::Osc2Level, v)),
        "o3l" => push_cmd(SynthCommand::Param(SynthParam::Osc3Level, v)),
        "o4l" => push_cmd(SynthCommand::Param(SynthParam::Osc4Level, v)),
        "o1pw" => push_cmd(SynthCommand::Param(SynthParam::Osc1Pw, v)),
        "o2pw" => push_cmd(SynthCommand::Param(SynthParam::Osc2Pw, v)),
        "o3pw" => push_cmd(SynthCommand::Param(SynthParam::Osc3Pw, v)),
        "o4pw" => push_cmd(SynthCommand::Param(SynthParam::Osc4Pw, v)),
        "o2s" => push_cmd(SynthCommand::Param(SynthParam::Osc2Semitones, v)),
        "o2c" => push_cmd(SynthCommand::Param(SynthParam::Osc2Cents, v)),
        "o3s" => push_cmd(SynthCommand::Param(SynthParam::Osc3Semitones, v)),
        "o3c" => push_cmd(SynthCommand::Param(SynthParam::Osc3Cents, v)),
        "o4s" => push_cmd(SynthCommand::Param(SynthParam::Osc4Semitones, v)),
        "o4c" => push_cmd(SynthCommand::Param(SynthParam::Osc4Cents, v)),
        "o1pn" => push_cmd(SynthCommand::Param(SynthParam::Osc1Pan, v)),
        "o2pn" => push_cmd(SynthCommand::Param(SynthParam::Osc2Pan, v)),
        "o3pn" => push_cmd(SynthCommand::Param(SynthParam::Osc3Pan, v)),
        "o4pn" => push_cmd(SynthCommand::Param(SynthParam::Osc4Pan, v)),
        "fmode" => push_cmd(SynthCommand::FilterMode(v as u8)),
        "cut" => push_cmd(SynthCommand::Param(SynthParam::FilterCutoff, v)),
        "res" => push_cmd(SynthCommand::Param(SynthParam::FilterResonance, v)),
        "fenv" => push_cmd(SynthCommand::Param(SynthParam::FilterEnvAmount, v)),
        "fatk" => push_cmd(SynthCommand::Param(SynthParam::FilterAttack, v)),
        "fdec" => push_cmd(SynthCommand::Param(SynthParam::FilterDecay, v)),
        "fsus" => push_cmd(SynthCommand::Param(SynthParam::FilterSustain, v)),
        "frel" => push_cmd(SynthCommand::Param(SynthParam::FilterRelease, v)),
        "wth" => push_cmd(SynthCommand::Param(SynthParam::StereoWidth, v)),
        "atk" => push_cmd(SynthCommand::Param(SynthParam::Attack, v)),
        "dec" => push_cmd(SynthCommand::Param(SynthParam::Decay, v)),
        "sus" => push_cmd(SynthCommand::Param(SynthParam::Sustain, v)),
        "rel" => push_cmd(SynthCommand::Param(SynthParam::Release, v)),
        "prt" => push_cmd(SynthCommand::Param(SynthParam::PortamentoTime, v)),
        "e3a" => push_cmd(SynthCommand::Env3Attack(v)),
        "e3d" => push_cmd(SynthCommand::Env3Decay(v)),
        "e3s" => push_cmd(SynthCommand::Env3Sustain(v)),
        "e3r" => push_cmd(SynthCommand::Env3Release(v)),
        "mw" => push_cmd(SynthCommand::ModWheel(v)),
        "ldst" => push_cmd(SynthCommand::Param(SynthParam::LfoDestination, v)),
        "lwf" => push_cmd(SynthCommand::Param(SynthParam::LfoWaveform, v)),
        "lrt" => push_cmd(SynthCommand::Param(SynthParam::LfoRate, v)),
        "ldep" => push_cmd(SynthCommand::Param(SynthParam::LfoDepth, v)),
        "l2wf" => push_cmd(SynthCommand::Lfo2Wf(v as u8)),
        "l2rt" => push_cmd(SynthCommand::Lfo2Rate(v)),
        "l2dep" => push_cmd(SynthCommand::Lfo2Depth(v)),
        "bpm" => push_cmd(SynthCommand::Bpm(v)),
        "l1_s" => push_cmd(SynthCommand::Lfo1Sync(v != 0.0)),
        "l1_b" => push_cmd(SynthCommand::Lfo1Beats(v)),
        "l2_s" => push_cmd(SynthCommand::Lfo2Sync(v != 0.0)),
        "l2_b" => push_cmd(SynthCommand::Lfo2Beats(v)),
        "dl_s" => push_cmd(SynthCommand::DelaySync(v != 0.0)),
        "dl_b" => push_cmd(SynthCommand::DelayBeats(v)),
        "dt_e" => push_cmd(SynthCommand::DistEnable(v != 0.0)),
        "dt_t" => push_cmd(SynthCommand::DistType(v as u8)),
        "dt_d" => push_cmd(SynthCommand::DistDrive(v)),
        "dt_m" => push_cmd(SynthCommand::DistMix(v)),
        "ch_r" => push_cmd(SynthCommand::ChorusRate(v)),
        "ch_d" => push_cmd(SynthCommand::ChorusDepth(v)),
        "ch_m" => push_cmd(SynthCommand::ChorusMix(v)),
        "dl_t" => push_cmd(SynthCommand::DelayTime(v)),
        "dl_fb" => push_cmd(SynthCommand::DelayFb(v)),
        "dl_m" => push_cmd(SynthCommand::DelayMix(v)),
        "ar_e" => push_cmd(SynthCommand::ArpOn(v != 0.0)),
        "ar_m" => push_cmd(SynthCommand::ArpMode(v as u8)),
        "ar_r" => push_cmd(SynthCommand::ArpRate(v)),
        "ar_o" => push_cmd(SynthCommand::ArpOct(v as u8)),
        "ar_h" => push_cmd(SynthCommand::ArpHold(v != 0.0)),
        "rv_r" => push_cmd(SynthCommand::RevRoom(v)),
        "rv_d" => push_cmd(SynthCommand::RevDamp(v)),
        "rv_m" => push_cmd(SynthCommand::RevMix(v)),
        "vol" => push_cmd(SynthCommand::Param(SynthParam::MasterVolume, v)),
        "poly" => push_cmd(SynthCommand::PolyMode(v != 0.0)),
        "uni" => push_cmd(SynthCommand::Unison(v as u8)),
        "udet" => push_cmd(SynthCommand::UniDetune(v)),
        "uspr" => push_cmd(SynthCommand::UniSpread(v)),
        "pcap" => push_cmd(SynthCommand::VoiceCap(v as u8)),
        "srate" => push_cmd(SynthCommand::SynthHalf(v != 0.0)),
        "wstop" => {
            // RF-interference diagnostic: bounce the radio for N seconds —
            // if the periodic click in the audio path vanishes while the
            // radio is down, it's TX-burst coupling, not a DSP artifact.
            // The radio restarts automatically; the UI may need a refresh.
            let secs = v.clamp(1.0, 60.0);
            let _ = std::thread::Builder::new()
                .stack_size(4096)
                .spawn(move || {
                    std::thread::sleep(std::time::Duration::from_millis(500));
                    unsafe {
                        esp_wifi_stop();
                        std::thread::sleep(std::time::Duration::from_secs_f32(secs));
                        esp_wifi_start();
                        esp_wifi_connect();
                    }
                });
        }
        "wpow" => {
            // TX power in quarter-dBm units (8 = 2 dBm .. 80 = 20 dBm).
            unsafe {
                esp_wifi_set_max_tx_power(v.clamp(4.0, 80.0) as i8);
            }
        }
        "dl" => {
            // Software download-mode entry: RTC_CNTL_FORCE_DOWNLOAD_BOOT
            // makes the NEXT reset boot into the UART downloader regardless
            // of GPIO0 — the BOOT button never needs touching. The flag is
            // cleared by the ROM after one download boot, so the following
            // reset boots the app normally again. Small delay lets this
            // HTTP response flush first.
            let _ = std::thread::Builder::new()
                .stack_size(2048)
                .spawn(|| {
                    std::thread::sleep(std::time::Duration::from_millis(300));
                    unsafe {
                        let reg = RTC_CNTL_OPTION1_REG as *mut u32;
                        let cur = core::ptr::read_volatile(reg);
                        core::ptr::write_volatile(
                            reg,
                            cur | (RTC_CNTL_FORCE_DOWNLOAD_BOOT_V
                                << RTC_CNTL_FORCE_DOWNLOAD_BOOT_S),
                        );
                        esp_restart();
                    }
                });
        }
        _ => {}
    }
}

// ---------------------------------------------------------------------------
// NVS Wi-Fi credentials load/save + AP+STA Mixed mode configuration
// ---------------------------------------------------------------------------

fn url_decode(s: &str) -> String {
    let mut out = Vec::with_capacity(s.len());
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'+' {
            out.push(b' ');
            i += 1;
        } else if b[i] == b'%' && i + 2 < b.len() {
            if let Ok(hex) = core::str::from_utf8(&b[i + 1..i + 3]) {
                if let Ok(v) = u8::from_str_radix(hex, 16) {
                    out.push(v);
                    i += 3;
                    continue;
                }
            }
            out.push(b[i]);
            i += 1;
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

unsafe fn nvs_load_wifi() -> (String, String) {
    let mut h: nvs_handle_t = 0;
    if nvs_open(c"wifi".as_ptr(), nvs_open_mode_t_NVS_READWRITE, &mut h) != ESP_OK {
        return (String::new(), String::new());
    }
    let mut ssid_buf = [0u8; 33];
    let mut pass_buf = [0u8; 65];
    let mut slen = ssid_buf.len();
    let mut plen = pass_buf.len();
    let ssid = if nvs_get_str(h, c"ssid".as_ptr(), ssid_buf.as_mut_ptr() as *mut _, &mut slen) == ESP_OK {
        CStr::from_ptr(ssid_buf.as_ptr() as *const _).to_string_lossy().into_owned()
    } else {
        String::new()
    };
    let pass = if nvs_get_str(h, c"pass".as_ptr(), pass_buf.as_mut_ptr() as *mut _, &mut plen) == ESP_OK {
        CStr::from_ptr(pass_buf.as_ptr() as *const _).to_string_lossy().into_owned()
    } else {
        String::new()
    };
    nvs_close(h);
    (ssid, pass)
}

unsafe fn nvs_save_wifi(ssid: &str, pass: &str) {
    let mut h: nvs_handle_t = 0;
    if nvs_open(c"wifi".as_ptr(), nvs_open_mode_t_NVS_READWRITE, &mut h) != ESP_OK {
        return;
    }
    if let Ok(cs) = CString::new(ssid) {
        let _ = nvs_set_str(h, c"ssid".as_ptr(), cs.as_ptr());
    }
    if let Ok(cp) = CString::new(pass) {
        let _ = nvs_set_str(h, c"pass".as_ptr(), cp.as_ptr());
    }
    let _ = nvs_commit(h);
    nvs_close(h);
}

fn ap_config() -> AccessPointConfiguration {
    AccessPointConfiguration {
        ssid: AP_SSID.try_into().unwrap(),
        channel: 1,
        auth_method: AuthMethod::None,
        max_connections: 4,
        ..Default::default()
    }
}

fn configure_wifi_sta(ssid: &str, pass: &str) {
    if let Ok(mut guard) = SAVED_SSID.lock() {
        *guard = ssid.to_string();
    }
    let Ok(mut guard) = WIFI.lock() else { return };
    let Some(wifi) = guard.as_mut() else { return };
    if ssid.is_empty() {
        let _ = wifi.set_configuration(&WifiConfig::AccessPoint(ap_config()));
        let _ = wifi.start();
    } else {
        let Ok(ssid_hs) = ssid.try_into() else { return };
        let Ok(pass_hs) = pass.try_into() else { return };
        let auth = if pass.is_empty() {
            AuthMethod::None
        } else {
            AuthMethod::WPA2Personal
        };
        let sta_cfg = ClientConfiguration {
            ssid: ssid_hs,
            password: pass_hs,
            auth_method: auth,
            ..Default::default()
        };
        if wifi
            .set_configuration(&WifiConfig::Mixed(sta_cfg, ap_config()))
            .is_ok()
        {
            let _ = wifi.start();
            let _ = wifi.connect();
            info!("web: connecting STA to '{}' (AP '{}' remains active)", ssid, AP_SSID);
        }
    }
    unsafe {
        esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE);
    }
}

fn wifi_status_json() -> String {
    let ssid = SAVED_SSID.lock().map(|s| s.clone()).unwrap_or_default();
    let sta_ip = WIFI
        .lock()
        .ok()
        .and_then(|g| g.as_ref()?.sta_netif().get_ip_info().ok())
        .map(|info| info.ip.to_string())
        .unwrap_or_else(|| "0.0.0.0".to_string());
    format!(
        "{{\"ap\":\"192.168.71.1\",\"ssid\":\"{}\",\"ip\":\"{}\"}}",
        ssid, sta_ip
    )
}

// ---------------------------------------------------------------------------
// Captive Portal UDP Port 53 DNS Responder (resolves all A queries -> 192.168.71.1)
// ---------------------------------------------------------------------------

fn start_captive_dns() {
    let _ = std::thread::Builder::new()
        .name("captive_dns".into())
        .stack_size(4096)
        .spawn(|| {
            let Ok(sock) = UdpSocket::bind("0.0.0.0:53") else {
                log::warn!("web: could not bind UDP :53 for captive DNS");
                return;
            };
            let mut buf = [0u8; 512];
            loop {
                let Ok((len, src)) = sock.recv_from(&mut buf) else {
                    continue;
                };
                // Minimum DNS header (12 bytes) + question (5 bytes)
                if len < 17 || (buf[2] & 0x80) != 0 {
                    continue;
                }
                // Find end of first QNAME + QTYPE(2) + QCLASS(2)
                let mut pos = 12;
                while pos < len && buf[pos] != 0 {
                    let l = buf[pos] as usize;
                    if (l & 0xC0) == 0xC0 {
                        pos += 1;
                        break;
                    }
                    pos += 1 + l;
                }
                let q_end = pos + 5;
                if q_end > len || q_end + 16 > buf.len() {
                    continue;
                }
                // Build DNS response in place
                buf[2] = 0x81; // QR=1, RD=1
                buf[3] = 0x80; // RA=1, RCODE=0
                buf[4] = 0x00;
                buf[5] = 0x01; // QDCOUNT = 1
                buf[6] = 0x00;
                buf[7] = 0x01; // ANCOUNT = 1
                buf[8] = 0x00;
                buf[9] = 0x00; // NSCOUNT = 0
                buf[10] = 0x00;
                buf[11] = 0x00; // ARCOUNT = 0
                let ans: [u8; 16] = [
                    0xC0, 0x0C, // Name pointer to offset 12
                    0x00, 0x01, // Type A
                    0x00, 0x01, // Class IN
                    0x00, 0x00, 0x00, 0x3C, // TTL 60s
                    0x00, 0x04, // RDLENGTH 4
                    AP_IP[0], AP_IP[1], AP_IP[2], AP_IP[3],
                ];
                buf[q_end..q_end + 16].copy_from_slice(&ans);
                let _ = sock.send_to(&buf[..q_end + 16], src);
            }
        });
}

// ---------------------------------------------------------------------------
// Raw ESP-IDF HTTP + WebSocket handlers
// ---------------------------------------------------------------------------

unsafe extern "C" fn index_get(req: *mut httpd_req_t) -> esp_err_t {
    httpd_resp_set_type(req, c"text/html; charset=utf-8".as_ptr());
    httpd_resp_send(req, INDEX_HTML.as_ptr() as *const _, INDEX_HTML.len() as isize)
}

unsafe extern "C" fn captive_404(req: *mut httpd_req_t, _err: httpd_err_code_t) -> esp_err_t {
    httpd_resp_set_status(req, c"302 Found".as_ptr());
    httpd_resp_set_hdr(req, c"Location".as_ptr(), c"http://192.168.71.1/".as_ptr());
    httpd_resp_send(req, c"Redirecting to RustInSynth".as_ptr(), 26)
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
    control::set_program_change(idx as u8);
    let body = PRESET_DETAILS_JSON
        .get()
        .and_then(|v| v.get(idx))
        .map(|s| s.as_str())
        .unwrap_or("{}");
    httpd_resp_set_type(req, c"application/json".as_ptr());
    httpd_resp_send(req, body.as_ptr() as *const _, body.len() as isize)
}

unsafe extern "C" fn wifi_get(req: *mut httpd_req_t) -> esp_err_t {
    let uri = CStr::from_ptr((*req).uri.as_ptr()).to_str().unwrap_or("");
    if let Some((_, q)) = uri.split_once('?') {
        let mut ssid = None;
        let mut pass = String::new();
        for pair in q.split('&') {
            if let Some((k, v)) = pair.split_once('=') {
                match k {
                    "ssid" => ssid = Some(url_decode(v)),
                    "pass" => pass = url_decode(v),
                    _ => {}
                }
            }
        }
        if let Some(s) = ssid {
            nvs_save_wifi(&s, &pass);
            configure_wifi_sta(&s, &pass);
        }
    }
    let body = wifi_status_json();
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

/// Runs in the httpd task context (via `httpd_queue_work`): send a
/// server-initiated WS PING to `arg` (fd as pointer). A failed send means the
/// session is wedged — trigger an async close to free the slot. The client's
/// protocol-level PONG also refreshes the session's LRU counter so an active
/// WS is never the eviction victim.
unsafe extern "C" fn ws_ping_work(arg: *mut core::ffi::c_void) {
    let hd = HTTPD_HANDLE.load(Ordering::Acquire) as httpd_handle_t;
    if hd.is_null() {
        return;
    }
    let fd = arg as i32;
    // The fd may have been closed/reused since enumeration — only ping if it
    // still belongs to a WS session, never inject frames into HTTP traffic.
    if httpd_ws_get_fd_info(hd, fd) != httpd_ws_client_info_t_HTTPD_WS_CLIENT_WEBSOCKET {
        return;
    }
    let mut f: httpd_ws_frame_t = core::mem::zeroed();
    f.final_ = true;
    f.type_ = httpd_ws_type_t_HTTPD_WS_TYPE_PING;
    if httpd_ws_send_frame_async(hd, fd, &mut f) != ESP_OK {
        let _ = httpd_sess_trigger_close(hd, fd);
    }
}

/// Every 10 s enumerate httpd sessions and queue a WS PING for each live
/// WebSocket client. Combined with TCP keep-alive this bounds stale-session
/// lifetime (sleeping phones, dropped AP associations) to ~10-25 s, which is
/// what kept all server sockets occupied and froze the UI after a while.
fn start_ws_watchdog() {
    let _ = std::thread::Builder::new()
        .name("ws_ping".into())
        .stack_size(3072)
        .spawn(|| loop {
            std::thread::sleep(std::time::Duration::from_secs(10));
            let hd = HTTPD_HANDLE.load(Ordering::Acquire) as httpd_handle_t;
            if hd.is_null() {
                continue;
            }
            unsafe {
                let mut fds = [0i32; MAX_HTTPD_SESSIONS];
                let mut n = fds.len();
                if httpd_get_client_list(hd, &mut n, fds.as_mut_ptr()) != ESP_OK {
                    continue;
                }
                // DIAG: watch internal SRAM — lwIP pbufs live there; a leak
                // starves multi-buffer (large-frame) TX allocs first.
                log::info!(
                    "mon: heap int={} total={} | httpd sessions={}",
                    heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                    esp_get_free_heap_size(),
                    n
                );
                for &fd in &fds[..n] {
                    if httpd_ws_get_fd_info(hd, fd)
                        == httpd_ws_client_info_t_HTTPD_WS_CLIENT_WEBSOCKET
                    {
                        let _ = httpd_queue_work(
                            hd,
                            Some(ws_ping_work),
                            fd as *mut core::ffi::c_void,
                        );
                    }
                }
            }
        });
}

unsafe extern "C" fn ws_get(req: *mut httpd_req_t) -> esp_err_t {
    if (*req).method == http_method_HTTP_GET as i32 {
        return ESP_OK; // completes the WebSocket handshake
    }
    let mut pkt: httpd_ws_frame_t = core::mem::zeroed();

    // Fetch frame header + payload length
    let ret = httpd_ws_recv_frame(req, &mut pkt, 0);
    if ret != ESP_OK {
        return ret;
    }

    // Reject oversized frames by closing the session so unread bytes never
    // corrupt the stream. 1 KiB headroom fits a full batch of ~60 params.
    if pkt.len > 1023 {
        return ESP_FAIL;
    }

    let mut buf = [0u8; 1024];
    if pkt.len > 0 {
        pkt.payload = buf.as_mut_ptr();
        let ret = httpd_ws_recv_frame(req, &mut pkt, pkt.len);
        if ret != ESP_OK {
            return ret;
        }
    }

    match pkt.type_ {
        x if x == httpd_ws_type_t_HTTPD_WS_TYPE_CLOSE => {
            // Client closed the WebSocket — return ESP_FAIL so httpd frees the socket!
            return ESP_FAIL;
        }
        x if x == httpd_ws_type_t_HTTPD_WS_TYPE_PING => {
            pkt.type_ = httpd_ws_type_t_HTTPD_WS_TYPE_PONG;
            let _ = httpd_ws_send_frame(req, &mut pkt);
            return ESP_OK;
        }
        x if x == httpd_ws_type_t_HTTPD_WS_TYPE_TEXT => {
            if let Ok(msg) = core::str::from_utf8(&buf[..pkt.len]) {
                if msg == "ping" {
                    let pong = b"pong";
                    let mut out: httpd_ws_frame_t = core::mem::zeroed();
                    out.final_ = true;
                    out.type_ = httpd_ws_type_t_HTTPD_WS_TYPE_TEXT;
                    out.payload = pong.as_ptr() as *mut u8;
                    out.len = pong.len();
                    let _ = httpd_ws_send_frame(req, &mut out);
                } else if let Some(rest) = msg.strip_prefix("preset:") {
                    let idx = rest.trim().parse::<usize>().unwrap_or(0);
                    control::set_program_change(idx as u8);
                    if let Some(body) = PRESET_DETAILS_JSON.get().and_then(|v| v.get(idx)) {
                        let mut out: httpd_ws_frame_t = core::mem::zeroed();
                        out.final_ = true;
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
        _ => {}
    }
    ESP_OK
}

/// Pre-build preset directory & per-preset UI JSON in PSRAM.
pub fn init_presets_json(bank: &[Preset]) {
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
}

/// Start the SoftAP (+ optional STA) + Captive DNS + HTTP/WS server on Core 0.
pub fn start(modem: Modem<'static>) -> anyhow::Result<()> {

    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let mut wifi = EspWifi::new(modem, sysloop, Some(nvs))?;
    wifi.set_configuration(&WifiConfig::AccessPoint(ap_config()))?;
    wifi.start()?;
    // The AP netif is created with the stock 192.168.4.1 — re-assign it to
    // AP_IP so the captive DNS answers, redirects and the OLED status string
    // actually describe the real interface address.
    unsafe {
        let ap = wifi.ap_netif().handle();
        let _ = esp_netif_dhcps_stop(ap);
        let ip_info = esp_netif_ip_info_t {
            ip: esp_ip4_addr_t {
                addr: u32::from_le_bytes(AP_IP),
            },
            netmask: esp_ip4_addr_t {
                addr: u32::from_le_bytes([255, 255, 255, 0]),
            },
            gw: esp_ip4_addr_t {
                addr: u32::from_le_bytes(AP_IP),
            },
        };
        esp!(esp_netif_set_ip_info(ap, &ip_info))?;
        esp!(esp_netif_dhcps_start(ap))?;
    }
    unsafe {
        esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE);
        // TX bursts (AP beacons ~10 Hz + traffic) couple into the DAC analog
        // path as an audible periodic click (confirmed: silent with radio
        // off). 10 dBm is plenty for a desk-range UI and ~10x less burst
        // energy than the 20 dBm default. `wpow` can tune it at runtime.
        esp_wifi_set_max_tx_power(40);
        // Wi-Fi config persists in NVS — re-assert the 11bgn protocol bitmap
        // explicitly so an earlier diagnostic b/g-only setting can't stick.
        let _ = esp_wifi_set_protocol(
            wifi_interface_t_WIFI_IF_AP,
            (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N) as u8,
        );
    }
    if let Ok(mut g) = WIFI.lock() {
        *g = Some(wifi);
    }

    // Restore saved STA credentials from NVS if present
    let (saved_ssid, saved_pass) = unsafe { nvs_load_wifi() };
    if !saved_ssid.is_empty() {
        configure_wifi_sta(&saved_ssid, &saved_pass);
    }

    start_captive_dns();

    // mDNS: rustinsynth.local resolves on the SoftAP AND on the home LAN
    // once STA credentials connect — the captive DNS answers that name on
    // the AP anyway; this is what matters on a real LAN, where the plain
    // DNS responder is never reached. mdns attaches to both netifs and
    // re-announces when addresses change.
    #[cfg(esp_idf_comp_espressif__mdns_enabled)]
    unsafe {
        let _ = mdns_init();
        let _ = mdns_hostname_set(c"rustinsynth".as_ptr());
        let _ = mdns_instance_name_set(c"Rust In Synth".as_ptr());
        let _ = mdns_service_add(
            core::ptr::null_mut(),
            c"_http".as_ptr(),
            c"_tcp".as_ptr(),
            80,
            core::ptr::null_mut(),
            0,
        );
        info!("web: mDNS up as rustinsynth.local (_http._tcp:80)");
    }

    unsafe {
        let cfg = httpd_config_t {
            task_priority: 6,
            stack_size: 8192,
            core_id: 0,
            task_caps: MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
            server_port: 80,
            ctrl_port: 32768,
            max_open_sockets: MAX_HTTPD_SESSIONS as u16,
            max_uri_handlers: 12,
            max_resp_headers: 8,
            backlog_conn: 5,
            lru_purge_enable: true,
            recv_wait_timeout: 2,
            send_wait_timeout: 3,
            // TCP keep-alive on every accepted socket: dead peers (phone
            // slept/roamed mid-session) are detected in ~10+2x5 s and their
            // session slots freed instead of lingering forever.
            keep_alive_enable: true,
            keep_alive_idle: 10,
            keep_alive_interval: 5,
            keep_alive_count: 2,
            ..Default::default()
        };
        let mut srv: httpd_handle_t = core::ptr::null_mut();
        esp!(httpd_start(&mut srv, &cfg))?;
        HTTPD_HANDLE.store(srv as usize, Ordering::Release);
        start_ws_watchdog();

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
                uri: c"/wifi".as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(wifi_get),
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
                // httpd reads the frame's first byte itself, then only calls
                // the handler for data frames UNLESS this flag is set. Without
                // it, a client PING/PONG (browsers auto-PONG our watchdog's
                // pings) leaves its remaining bytes in the socket, desyncs the
                // stream, and the next parse fails "not properly masked".
                handle_ws_control_frames: true,
                ..Default::default()
            },
            // Captive portal probe paths for Android / iOS / Windows
            httpd_uri_t {
                uri: c"/generate_204".as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(index_get),
                ..Default::default()
            },
            httpd_uri_t {
                uri: c"/hotspot-detect.html".as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(index_get),
                ..Default::default()
            },
            httpd_uri_t {
                uri: c"/connecttest.txt".as_ptr(),
                method: http_method_HTTP_GET,
                handler: Some(index_get),
                ..Default::default()
            },
        ];
        for u in &uris {
            esp!(httpd_register_uri_handler(srv, u))?;
        }
        let _ = httpd_register_err_handler(
            srv,
            httpd_err_code_t_HTTPD_404_NOT_FOUND,
            Some(captive_404),
        );
    }

    info!("web: SoftAP '{}' + Captive Portal ready at http://192.168.71.1", AP_SSID);
    Ok(())
}
