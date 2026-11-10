//! USB-MIDI host — Rust port of serviSynth's `usb_midi.c`.
//!
//! Installs the ESP-IDF USB host library, watches for a MIDI Streaming
//! interface (class 0x01 / subclass 0x03), and reads 4-byte USB-MIDI
//! packets on a bulk/interrupt IN endpoint. Parsed events go into a
//! lock-free SPSC ring consumed by the audio task — no locks, no allocs,
//! no blocking on the realtime path.
//!
//! Two FreeRTOS tasks on core 0 (the audio task owns core 1):
//!   usb_lib    — usb_host_lib_handle_events() pump
//!   usb_client — client event handling + MIDI interface discovery

use core::sync::atomic::{AtomicBool, Ordering};
use esp_idf_sys::*;
use log::info;
use rinsynth_esp32::control;
use rinsynth_model::event::SynthEvent;

// USB-MIDI Code Index Numbers (low nibble of packet byte 0)
const CIN_SYS_COMMON_1B: u8 = 0x5;
const CIN_NOTE_OFF: u8 = 0x8;
const CIN_NOTE_ON: u8 = 0x9;
const CIN_CONTROL: u8 = 0xB;
const CIN_PROGRAM: u8 = 0xC;
const CIN_PITCH_BEND: u8 = 0xE;
const CIN_SINGLE_BYTE: u8 = 0xF;

const XFER_SIZE: usize = 256;

static DEV_CONNECTED: AtomicBool = AtomicBool::new(false);

/// True while a class-compliant USB-MIDI device is attached and claimed.
pub fn device_connected() -> bool {
    DEV_CONNECTED.load(Ordering::Relaxed)
}

// ---------------------------------------------------------------------------
// USB host plumbing — state is only touched by the usb_client task, so a
// plain static mut is safe here (single-threaded access by construction).
// ---------------------------------------------------------------------------

struct Host {
    client: usb_host_client_handle_t,
    dev: usb_device_handle_t,
    xfer: *mut usb_transfer_t,
    intf_num: u8,
    alt_setting: u8,
}

static mut HOST: Host = Host {
    client: core::ptr::null_mut(),
    dev: core::ptr::null_mut(),
    xfer: core::ptr::null_mut(),
    intf_num: 0,
    alt_setting: 0,
};

// MIDI Timing Clock (0xF8, 24 PPQN) state — only touched by `usb_client` task.
static mut CLOCK_TICKS: u8 = 0;
static mut CLOCK_LAST_US: i64 = 0;
static mut CLOCK_BPM_EMA: f32 = 120.0;

unsafe fn on_midi_clock_tick() {
    let now = esp_timer_get_time();
    let prev = CLOCK_LAST_US;
    // Reset window on first tick or after >2 s gap (stopped clock / new song)
    if prev == 0 || (now - prev) > 2_000_000 {
        CLOCK_LAST_US = now;
        CLOCK_TICKS = 0;
        return;
    }
    CLOCK_TICKS += 1;
    // Measure every 6 ticks (1/16 note = 0.25 beat) for low jitter + fast lock
    if CLOCK_TICKS >= 6 {
        let dt_us = (now - prev) as f32;
        CLOCK_LAST_US = now;
        CLOCK_TICKS = 0;
        if dt_us > 25_000.0 && dt_us < 750_000.0 {
            let inst_bpm = (15_000_000.0 / dt_us).clamp(20.0, 300.0);
            CLOCK_BPM_EMA = if (CLOCK_BPM_EMA - inst_bpm).abs() > 15.0 {
                inst_bpm
            } else {
                0.75 * CLOCK_BPM_EMA + 0.25 * inst_bpm
            };
            control::set_clock_bpm(CLOCK_BPM_EMA);
        }
    }
}

unsafe fn parse_packet(p: &[u8]) {
    let cin = p[0] & 0x0F;
    let status = p[1];
    let d1 = p[2];
    let d2 = p[3];
    if (cin == CIN_SINGLE_BYTE || cin == CIN_SYS_COMMON_1B) && status == 0xF8 {
        on_midi_clock_tick();
        return;
    }
    let ev = match cin {
        CIN_NOTE_ON if d2 > 0 => SynthEvent::note_on_midi(d1, d2),
        CIN_NOTE_ON | CIN_NOTE_OFF => SynthEvent::note_off(d1),
        CIN_CONTROL => {
            if d1 == 70 {
                // CC 70 (Sound Variation) -> select preset index
                control::set_program_change(d2);
            }
            SynthEvent::control_change(d1, d2)
        }
        CIN_PROGRAM => {
            control::record_program_change(d1);
            return;
        }
        CIN_PITCH_BEND => SynthEvent::pitch_bend_midi(((d2 as u16) << 7) | d1 as u16),
        _ => return,
    };
    control::push_midi_event(ev);
}

unsafe extern "C" fn in_xfer_cb(xfer: *mut usb_transfer_t) {
    let x = &*xfer;
    if x.status == usb_transfer_status_t_USB_TRANSFER_STATUS_COMPLETED
        && x.actual_num_bytes >= 4
    {
        let data = core::slice::from_raw_parts(x.data_buffer, x.actual_num_bytes as usize);
        for pkt in data.chunks_exact(4) {
            if pkt[0] != 0 {
                parse_packet(pkt);
            }
        }
        usb_host_transfer_submit(xfer);
    }
    // Any other status (abort on disconnect, stall, ...) just drops the transfer
}

unsafe fn try_open_device(dev_addr: u8) {
    let h = &mut *(&raw mut HOST);
    if usb_host_device_open(h.client, dev_addr, &mut h.dev) != ESP_OK as i32 {
        log::warn!("usb-midi: device open failed");
        return;
    }

    let mut cfg: *const usb_config_desc_t = core::ptr::null();
    usb_host_get_active_config_descriptor(h.dev, &mut cfg);
    if cfg.is_null() {
        usb_host_device_close(h.client, h.dev);
        h.dev = core::ptr::null_mut();
        return;
    }

    // Walk descriptors for a MIDI streaming interface (class 0x01,
    // subclass 0x03), then its first IN endpoint.
    let mut offset: i32 = 0;
    let mut in_midi = false;
    let mut ep_in: u8 = 0;
    let mut desc = cfg as *const usb_standard_desc_t;
    let total_len = (*cfg).__bindgen_anon_1.wTotalLength;
    loop {
        desc = usb_parse_next_descriptor(desc, total_len, &mut offset);
        if desc.is_null() {
            break;
        }
        match (*desc).__bindgen_anon_1.bDescriptorType as u32 {
            USB_B_DESCRIPTOR_TYPE_INTERFACE => {
                let id = &(*(desc as *const usb_intf_desc_t)).__bindgen_anon_1;
                h.intf_num = id.bInterfaceNumber;
                h.alt_setting = id.bAlternateSetting;
                in_midi = id.bInterfaceClass == 0x01 && id.bInterfaceSubClass == 0x03;
            }
            USB_B_DESCRIPTOR_TYPE_ENDPOINT if in_midi => {
                let ed = &(*(desc as *const usb_ep_desc_t)).__bindgen_anon_1;
                if ed.bEndpointAddress as u32 & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK != 0 {
                    ep_in = ed.bEndpointAddress;
                    break;
                }
            }
            _ => {}
        }
    }

    if ep_in == 0 {
        info!("usb-midi: not a MIDI device (addr {})", dev_addr);
        usb_host_device_close(h.client, h.dev);
        h.dev = core::ptr::null_mut();
        return;
    }

    usb_host_interface_claim(h.client, h.dev, h.intf_num, h.alt_setting);
    if usb_host_transfer_alloc(XFER_SIZE, 0, &mut h.xfer) != ESP_OK as i32 {
        log::error!("usb-midi: transfer alloc failed");
        return;
    }
    (*h.xfer).device_handle = h.dev;
    (*h.xfer).bEndpointAddress = ep_in;
    (*h.xfer).callback = Some(in_xfer_cb);
    (*h.xfer).num_bytes = XFER_SIZE as i32;
    usb_host_transfer_submit(h.xfer);
    DEV_CONNECTED.store(true, Ordering::Relaxed);

    info!(
        "usb-midi: device ready — intf {} alt {}, IN ep 0x{:02x}",
        h.intf_num, h.alt_setting, ep_in
    );
}

unsafe extern "C" fn client_event_cb(
    msg: *const usb_host_client_event_msg_t,
    _arg: *mut core::ffi::c_void,
) {
    let m = &*msg;
    match m.event {
        x if x == usb_host_client_event_t_USB_HOST_CLIENT_EVENT_NEW_DEV => {
            let addr = m.__bindgen_anon_1.new_dev.address;
            info!("usb-midi: device connected, addr {}", addr);
            try_open_device(addr);
        }
        x if x == usb_host_client_event_t_USB_HOST_CLIENT_EVENT_DEV_GONE => {
            info!("usb-midi: device gone");
            DEV_CONNECTED.store(false, Ordering::Relaxed);
            let h = &mut *(&raw mut HOST);
            if !h.dev.is_null() {
                usb_host_interface_release(h.client, h.dev, h.intf_num);
                usb_host_device_close(h.client, h.dev);
                h.dev = core::ptr::null_mut();
            }
            if !h.xfer.is_null() {
                usb_host_transfer_free(h.xfer);
                h.xfer = core::ptr::null_mut();
            }
        }
        _ => {}
    }
}

extern "C" fn usb_lib_task(_: *mut core::ffi::c_void) {
    loop {
        unsafe {
            let mut flags: u32 = 0;
            usb_host_lib_handle_events(u32::MAX, &mut flags);
        }
    }
}

extern "C" fn usb_client_task(_: *mut core::ffi::c_void) {
    unsafe {
        let mut cfg: usb_host_client_config_t = core::mem::zeroed();
        cfg.is_synchronous = false;
        cfg.max_num_event_msg = 8;
        cfg.__bindgen_anon_1.async_ = usb_host_client_config_t__bindgen_ty_1__bindgen_ty_1 {
            client_event_callback: Some(client_event_cb),
            callback_arg: core::ptr::null_mut(),
        };
        let h = &mut *(&raw mut HOST);
        if usb_host_client_register(&cfg, &mut h.client) != ESP_OK as i32 {
            log::error!("usb-midi: client register failed");
            return;
        }
        loop {
            usb_host_client_handle_events(h.client, u32::MAX);
        }
    }
}

/// Install the USB host stack and spawn the two service tasks on core 0.
pub fn start() {
    unsafe {
        let mut cfg: usb_host_config_t = core::mem::zeroed();
        cfg.skip_phy_setup = false;
        cfg.intr_flags = ESP_INTR_FLAG_LEVEL1 as i32;
        let err = usb_host_install(&cfg);
        if err != ESP_OK as i32 {
            log::error!("usb-midi: host install failed: {}", err);
            return;
        }

        esp_idf_hal::task::create(
            usb_lib_task,
            c"usb_lib",
            8192,
            core::ptr::null_mut(),
            10,
            Some(esp_idf_hal::cpu::Core::Core0),
        )
        .expect("usb_lib task");

        esp_idf_hal::task::create(
            usb_client_task,
            c"usb_client",
            8192,
            core::ptr::null_mut(),
            10,
            Some(esp_idf_hal::cpu::Core::Core0),
        )
        .expect("usb_client task");
    }

    info!("usb-midi: host installed, waiting for MIDI device...");
}
