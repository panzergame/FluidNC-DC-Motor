// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once
#include "Config.h"
#include "Driver/fluidnc_gpio.h"
// Quadrature encoder driver interface
#include <stdint.h>

struct EncoderSample {
    int32_t  count;
    // Timer value at the most recent edge, in timer_hz() ticks; only
    // meaningful when timed.  It belongs to count: no edge falls between the
    // two.
    uint32_t edge_time;
    bool     timed;
};

// x4 decoding: every edge on either input counts once.
class QuadratureEncoder {
public:
    // Pulses shorter than filter_ns are ignored; the hardware may round it
    // down to what its filter can express.
    QuadratureEncoder(pinnum_t a, pinnum_t b, uint32_t filter_ns);

    // False if the platform could not provide a counter, in which case
    // read() always returns 0.
    bool valid() const { return _unit >= 0; }

    // True if sample() can timestamp edges.  Timing hardware is scarcer than
    // counters, so an encoder can be valid without it.
    bool has_timing() const { return _timer_unit >= 0; }
    uint32_t timer_hz() const;

    // Counts since construction.  The hardware counter is narrower than the
    // result and is unwrapped here, so this must be called often enough that
    // the counter cannot move by more than half its range between calls.
    int32_t read();

    // read() plus the time of the edge that produced that count.  timed is
    // false without timing hardware, or if edges kept arriving while it tried
    // to take a consistent snapshot.
    EncoderSample sample();

private:
    int32_t _unit     = -1;
    int16_t _last_raw = 0;
    int32_t _count    = 0;

    int32_t  _timer_unit = -1;
    uint32_t _settle_us  = 1;
    uint32_t _last_cap_a = 0;
    uint32_t _last_cap_b = 0;
    uint32_t _edge_time  = 0;
};
