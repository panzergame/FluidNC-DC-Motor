// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
  Quadrature decoding on the ESP32 PCNT peripheral, through the legacy driver/pcnt.h API:
  Arduino-ESP32 2.x is on ESP-IDF 4.4, which predates driver/pulse_cnt.h.

  Edge timestamps come from MCPWM capture channels.  An encoder takes two channels of one
  group when it can, otherwise one from each of two groups, with the offset between their
  timers measured at setup.  Encoders beyond the channel count are counted but not timed.
*/
#include "Config.h"
#include "Driver/QuadratureEncoder.h"

#include <driver/pcnt.h>
#include <esp_rom_sys.h>
#include <soc/soc.h>  // APB_CLK_FREQ
#include <soc/soc_caps.h>

#if SOC_MCPWM_SUPPORTED
#    include <driver/mcpwm.h>
#    include <esp_idf_version.h>
#    include <hal/mcpwm_ll.h>
#    include <freertos/FreeRTOS.h>
#endif

// The counter resets to zero when it reaches either limit, so the raw value
// is effectively modulo this; symmetric limits keep the unwrap in read() the
// same in both directions.
static constexpr int16_t counter_limit = 32767;

// PCNT counts its glitch filter in APB clock cycles, in a 10-bit field.
static constexpr uint32_t apb_mhz        = APB_CLK_FREQ / 1000000;
static constexpr uint32_t max_filter_val = 1023;

static int32_t allocateUnit() {
    static int32_t nextUnit = 0;
    if (nextUnit >= PCNT_UNIT_MAX) {
        return -1;
    }
    return nextUnit++;
}

static void configureCounter(pcnt_unit_t u, pinnum_t a, pinnum_t b, uint32_t filter_val) {
    // Each input counts both of its edges, and the level of the other input
    // decides the direction - the standard x4 arrangement from the ESP-IDF
    // rotary encoder example.
    pcnt_config_t cfg = {};
    cfg.counter_h_lim = counter_limit;
    cfg.counter_l_lim = -counter_limit;
    cfg.unit          = u;

    cfg.pulse_gpio_num = a;
    cfg.ctrl_gpio_num  = b;
    cfg.channel        = PCNT_CHANNEL_0;
    cfg.pos_mode       = PCNT_COUNT_DEC;
    cfg.neg_mode       = PCNT_COUNT_INC;
    cfg.hctrl_mode     = PCNT_MODE_KEEP;
    cfg.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_unit_config(&cfg);

    cfg.pulse_gpio_num = b;
    cfg.ctrl_gpio_num  = a;
    cfg.channel        = PCNT_CHANNEL_1;
    cfg.pos_mode       = PCNT_COUNT_INC;
    cfg.neg_mode       = PCNT_COUNT_DEC;
    pcnt_unit_config(&cfg);

    if (filter_val) {
        pcnt_set_filter_value(u, uint16_t(filter_val));
        pcnt_filter_enable(u);
    } else {
        pcnt_filter_disable(u);
    }

    pcnt_counter_pause(u);
    pcnt_counter_clear(u);
    pcnt_counter_resume(u);
}

#if SOC_MCPWM_SUPPORTED
// The capture timer runs from APB regardless of the MCPWM group clock.
static constexpr uint32_t capture_timer_hz = APB_CLK_FREQ;

static int capChannelsUsed[SOC_MCPWM_GROUPS] = {};

struct CaptureInput {
    int unit;
    int channel;
};

// Both inputs in one group first, so that split encoders only appear once the
// groups are down to one spare channel each.
static bool allocateCapture(CaptureInput& a, CaptureInput& b) {
    for (int unit = 0; unit < SOC_MCPWM_GROUPS; ++unit) {
        if (capChannelsUsed[unit] + 2 <= SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER) {
            a = { unit, capChannelsUsed[unit]++ };
            b = { unit, capChannelsUsed[unit]++ };
            return true;
        }
    }
    int first = -1;
    for (int unit = 0; unit < SOC_MCPWM_GROUPS; ++unit) {
        if (capChannelsUsed[unit] < SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER) {
            if (first < 0) {
                first = unit;
            } else {
                a = { first, capChannelsUsed[first]++ };
                b = { unit, capChannelsUsed[unit]++ };
                return true;
            }
        }
    }
    return false;
}

static uint32_t captureValue(int unit, int channel) {
    return mcpwm_ll_capture_get_value(MCPWM_LL_GET_HW(unit), channel);
}

// Latches both capture timers by software back to back; the two register
// writes are a few APB cycles apart, which is far below an edge period.  An
// encoder edge landing between a trigger and its read corrupts one attempt,
// so the result must repeat before it is trusted.
static bool measureCaptureOffset(const CaptureInput& a, const CaptureInput& b, uint32_t& offset) {
    static portMUX_TYPE mux      = portMUX_INITIALIZER_UNLOCKED;
    uint32_t            previous = 0;
    for (int attempt = 0; attempt < 10; ++attempt) {
        portENTER_CRITICAL(&mux);
        mcpwm_ll_trigger_soft_capture(MCPWM_LL_GET_HW(a.unit), a.channel);
        mcpwm_ll_trigger_soft_capture(MCPWM_LL_GET_HW(b.unit), b.channel);
        uint32_t va = captureValue(a.unit, a.channel);
        uint32_t vb = captureValue(b.unit, b.channel);
        portEXIT_CRITICAL(&mux);

        uint32_t current = vb - va;
        if (attempt > 0) {
            int32_t diff = int32_t(current - previous);
            if (diff >= -2 && diff <= 2) {
                offset = current;
                return true;
            }
        }
        previous = current;
    }
    return false;
}

static bool enableCapture(const CaptureInput& input, pinnum_t pin) {
    auto unit    = mcpwm_unit_t(input.unit);
    auto channel = mcpwm_capture_channel_id_t(MCPWM_SELECT_CAP0 + input.channel);
    auto signal  = mcpwm_io_signals_t(MCPWM_CAP_0 + input.channel);
    // Only adds the pin to MCPWM's input; the PCNT routing is untouched.
    if (mcpwm_gpio_init(unit, signal, int(pin)) != ESP_OK) {
        return false;
    }
    mcpwm_capture_config_t cfg = {};
    cfg.cap_edge               = MCPWM_BOTH_EDGE;
    cfg.cap_prescale           = 1;
    if (mcpwm_capture_enable_channel(unit, channel, &cfg) != ESP_OK) {
        return false;
    }
#    if ESP_IDF_VERSION_MAJOR < 5
    // The latch is polled, so an interrupt on every edge would be pure load.
    // ESP-IDF 5 changed the LL API; there the driver's ISR just runs idle.
    mcpwm_ll_intr_enable_capture(MCPWM_LL_GET_HW(unit), uint32_t(channel), false);
#    endif
    return true;
}

static bool setupCapture(pinnum_t a, pinnum_t b, CaptureInput& cap_a, CaptureInput& cap_b, uint32_t& offset) {
    if (!allocateCapture(cap_a, cap_b)) {
        log_warn("Out of MCPWM capture channels for encoder timing; velocity will be count based");
        return false;
    }
    if (!enableCapture(cap_a, a) || !enableCapture(cap_b, b)) {
        log_error("Encoder timing setup failed; velocity will be count based");
        return false;
    }
    if (cap_a.unit != cap_b.unit) {
        if (!measureCaptureOffset(cap_a, cap_b, offset)) {
            log_error("Encoder capture timers could not be aligned; velocity will be count based");
            return false;
        }
        log_info("    Encoder timing split across MCPWM" << cap_a.unit << " and MCPWM" << cap_b.unit << ", offset " << offset);
    }
    return true;
}
#endif

QuadratureEncoder::QuadratureEncoder(pinnum_t a, pinnum_t b, uint32_t filter_ns) {
    int32_t unit = allocateUnit();
    if (unit < 0) {
        log_error("Out of PCNT units for quadrature encoder");
        return;
    }

    uint32_t filter_val = filter_ns * apb_mhz / 1000;
    if (filter_val > max_filter_val) {
        filter_val = max_filter_val;
    }
    configureCounter(pcnt_unit_t(unit), a, b, filter_val);

    _unit = unit;

    // PCNT counts an edge only once its glitch filter has passed it, while
    // the capture latches immediately; sample() waits this long so that a
    // captured edge is also counted.  The rounding up covers the input
    // synchronizer when the filter is off.
    _settle_us = (filter_val * 1000 / apb_mhz + 999) / 1000 + 1;

#if SOC_MCPWM_SUPPORTED
    CaptureInput cap_a, cap_b;
    if (!setupCapture(a, b, cap_a, cap_b, _b_offset)) {
        return;
    }
    _cap_unit_a = int8_t(cap_a.unit);
    _cap_chan_a = int8_t(cap_a.channel);
    _cap_unit_b = int8_t(cap_b.unit);
    _cap_chan_b = int8_t(cap_b.channel);
    _last_cap_a = captureValue(_cap_unit_a, _cap_chan_a);
    _last_cap_b = captureValue(_cap_unit_b, _cap_chan_b) - _b_offset;
    _edge_time  = _last_cap_a;
#endif
}

uint32_t QuadratureEncoder::timer_hz() const {
#if SOC_MCPWM_SUPPORTED
    return capture_timer_hz;
#else
    return 0;
#endif
}

int32_t QuadratureEncoder::read() {
    if (_unit < 0) {
        return 0;
    }

    int16_t raw = 0;
    pcnt_get_counter_value(pcnt_unit_t(_unit), &raw);

    // A delta of more than half the range can only be a pass through a limit
    // reset, since callers poll faster than that.
    int32_t delta = int32_t(raw) - int32_t(_last_raw);
    if (delta > counter_limit / 2) {
        delta -= counter_limit;
    } else if (delta < -counter_limit / 2) {
        delta += counter_limit;
    }
    _last_raw = raw;
    _count += delta;
    return _count;
}

EncoderSample QuadratureEncoder::sample() {
    EncoderSample s = { 0, _edge_time, false };
#if SOC_MCPWM_SUPPORTED
    if (_cap_unit_a >= 0) {
        auto cap_a = [this] { return captureValue(_cap_unit_a, _cap_chan_a); };
        auto cap_b = [this] { return captureValue(_cap_unit_b, _cap_chan_b) - _b_offset; };
        // An edge between reading the captures and reading the count would
        // pair a timestamp with the wrong count, so retry until the captures
        // hold still around the count.
        for (int tries = 0; tries < 3; ++tries) {
            uint32_t a = cap_a();
            uint32_t b = cap_b();
            esp_rom_delay_us(_settle_us);
            s.count = read();
            if (a != cap_a() || b != cap_b()) {
                continue;
            }
            // A channel that has not moved for a long time holds a value the
            // timer may have wrapped past, so only a channel that changed can
            // be the newest edge; the signed compare is safe once both did,
            // since they are then both recent.
            bool a_moved = a != _last_cap_a;
            bool b_moved = b != _last_cap_b;
            if (a_moved && b_moved) {
                _edge_time = int32_t(a - b) > 0 ? a : b;
            } else if (a_moved) {
                _edge_time = a;
            } else if (b_moved) {
                _edge_time = b;
            }
            _last_cap_a = a;
            _last_cap_b = b;
            s.edge_time = _edge_time;
            s.timed     = true;
            return s;
        }
        // _last_cap_* stay as they were, so the next sample still sees
        // these edges as new.
        return s;
    }
#endif
    s.count = read();
    return s;
}
