// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once
#include "Config.h"
#include "Driver/fluidnc_gpio.h"
// Quadrature encoder driver interface
#include <stdint.h>

struct EncoderSample {
    int32_t  count;
    uint32_t edge_time;  // timer_hz() ticks at the edge that produced count; only if timed
    bool     timed;
};

// x4 decoding: every edge on either input counts once.
class QuadratureEncoder {
public:
    QuadratureEncoder(pinnum_t a, pinnum_t b, uint32_t filter_ns);  // filter_ns may be rounded down

    bool     valid() const { return _unit >= 0; }
    bool     has_timing() const { return _cap_unit_a >= 0; }  // timing hardware is scarcer than counters
    uint32_t timer_hz() const;

    // Must be called before the hardware counter can move by half its range.
    // timed is false without timing hardware, or if edges kept arriving during the snapshot.
    EncoderSample sample();

private:
    int32_t read();

    int32_t _unit     = -1;
    int16_t _last_raw = 0;
    int32_t _count    = 0;

    // A and B may be latched by different MCPWM groups; _b_offset maps B's timer onto A's.
    int8_t   _cap_unit_a = -1;
    int8_t   _cap_chan_a = 0;
    int8_t   _cap_unit_b = -1;
    int8_t   _cap_chan_b = 0;
    uint32_t _b_offset   = 0;
    uint32_t _settle_us  = 1;
    uint32_t _last_cap_a = 0;
    uint32_t _last_cap_b = 0;
    uint32_t _edge_time  = 0;
};
