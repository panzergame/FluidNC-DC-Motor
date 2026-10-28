// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
  Quadrature decoding on the ESP32 PCNT peripheral.

  This uses the legacy driver/pcnt.h API because the Arduino-ESP32 2.x build
  is on ESP-IDF 4.4, which predates driver/pulse_cnt.h.  The legacy API is
  still present, deprecated, in ESP-IDF 5.
*/
#include "Config.h"
#include "Driver/QuadratureEncoder.h"

#include <driver/pcnt.h>

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
