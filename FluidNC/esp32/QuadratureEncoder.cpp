// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
  Quadrature decoding on the ESP32 PCNT peripheral.

  This uses the legacy driver/pcnt.h API because the Arduino-ESP32 2.x build
  is on ESP-IDF 4.4, which predates driver/pulse_cnt.h.  The legacy API is
  still present, deprecated, in ESP-IDF 5.

  Edge timestamps come from an MCPWM capture unit: two of its channels latch
  the capture timer on every edge of A and of B.  The channels of a unit share
  one timer, so each timed encoder takes a whole unit, and encoders beyond the
  unit count are counted but not timed.
*/
#include "Config.h"
#include "Driver/QuadratureEncoder.h"

#include <driver/pcnt.h>
#include <esp_rom_sys.h>
#include <soc/soc_caps.h>

#if SOC_MCPWM_SUPPORTED
#    include <driver/mcpwm.h>
#    include <esp_idf_version.h>
#    if ESP_IDF_VERSION_MAJOR < 5
#        include <hal/mcpwm_ll.h>
#    endif
#endif

// The counter resets to zero when it reaches either limit, so the raw value
// is effectively modulo this; symmetric limits keep the unwrap in read() the
// same in both directions.
static constexpr int16_t counter_limit = 32767;

// PCNT counts its glitch filter in APB clock cycles, in a 10-bit field.
static constexpr uint32_t apb_mhz        = 80;
static constexpr uint32_t max_filter_val = 1023;

static int32_t allocateUnit() {
    static int32_t nextUnit = 0;
    if (nextUnit >= PCNT_UNIT_MAX) {
        return -1;
    }
    return nextUnit++;
}

#if SOC_MCPWM_SUPPORTED
// The capture timer runs from APB regardless of the MCPWM group clock.
static constexpr uint32_t capture_timer_hz = apb_mhz * 1000000;

static int32_t allocateTimerUnit() {
    static int32_t nextUnit = 0;
    if (nextUnit >= SOC_MCPWM_GROUPS) {
        return -1;
    }
    return nextUnit++;
}

static bool enableCapture(mcpwm_unit_t unit, mcpwm_capture_channel_id_t channel, mcpwm_io_signals_t signal, pinnum_t pin) {
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
#endif

QuadratureEncoder::QuadratureEncoder(pinnum_t a, pinnum_t b, uint32_t filter_ns) {
    int32_t unit = allocateUnit();
    if (unit < 0) {
        log_error("Out of PCNT units for quadrature encoder");
        return;
    }
    auto u = pcnt_unit_t(unit);

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

    uint32_t filter_val = filter_ns * apb_mhz / 1000;
    if (filter_val > max_filter_val) {
        filter_val = max_filter_val;
    }
    if (filter_val) {
        pcnt_set_filter_value(u, uint16_t(filter_val));
        pcnt_filter_enable(u);
    } else {
        pcnt_filter_disable(u);
    }

    pcnt_counter_pause(u);
    pcnt_counter_clear(u);
    pcnt_counter_resume(u);

    _unit = unit;

    // PCNT counts an edge only once its glitch filter has passed it, while
    // the capture latches immediately; sample() waits this long so that a
    // captured edge is also counted.  The rounding up covers the input
    // synchronizer when the filter is off.
    _settle_us = (filter_val * 1000 / apb_mhz + 999) / 1000 + 1;

#if SOC_MCPWM_SUPPORTED
    int32_t timer_unit = allocateTimerUnit();
    if (timer_unit < 0) {
        log_warn("Out of MCPWM units for encoder timing; velocity will be count based");
        return;
    }
    auto t = mcpwm_unit_t(timer_unit);
    if (!enableCapture(t, MCPWM_SELECT_CAP0, MCPWM_CAP_0, a) || !enableCapture(t, MCPWM_SELECT_CAP1, MCPWM_CAP_1, b)) {
        log_error("Encoder timing setup failed; velocity will be count based");
        return;
    }
    _last_cap_a  = mcpwm_capture_signal_get_value(t, MCPWM_SELECT_CAP0);
    _last_cap_b  = mcpwm_capture_signal_get_value(t, MCPWM_SELECT_CAP1);
    _edge_time   = _last_cap_a;
    _timer_unit  = timer_unit;
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
    if (_timer_unit >= 0) {
        auto unit = mcpwm_unit_t(_timer_unit);
        // An edge between reading the captures and reading the count would
        // pair a timestamp with the wrong count, so retry until the captures
        // hold still around the count.
        for (int tries = 0; tries < 3; ++tries) {
            uint32_t a = mcpwm_capture_signal_get_value(unit, MCPWM_SELECT_CAP0);
            uint32_t b = mcpwm_capture_signal_get_value(unit, MCPWM_SELECT_CAP1);
            esp_rom_delay_us(_settle_us);
            s.count = read();
            if (a != mcpwm_capture_signal_get_value(unit, MCPWM_SELECT_CAP0) || b != mcpwm_capture_signal_get_value(unit, MCPWM_SELECT_CAP1)) {
                continue;
            }
            // A channel that has not moved for a long time holds a value the
            // timer may have wrapped past, so only a channel that changed can
            // be the newest edge; the signed compare is safe once both did,
            // since they are then both recent.
            const bool a_moved = a != _last_cap_a;
            const bool b_moved = b != _last_cap_b;
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
