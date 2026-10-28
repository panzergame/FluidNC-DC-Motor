// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once
#include "Config.h"
#include "Driver/fluidnc_gpio.h"
// Quadrature encoder driver interface
#include <stdint.h>

// x4 decoding: every edge on either input counts once.
class QuadratureEncoder {
public:
    // Pulses shorter than filter_ns are ignored; the hardware may round it
    // down to what its filter can express.
    QuadratureEncoder(pinnum_t a, pinnum_t b, uint32_t filter_ns);

    // False if the platform could not provide a counter, in which case
    // read() always returns 0.
    bool valid() const { return _unit >= 0; }

    // Counts since construction.  The hardware counter is narrower than the
    // result and is unwrapped here, so this must be called often enough that
    // the counter cannot move by more than half its range between calls.
    int32_t read();

private:
    int32_t _unit     = -1;
    int16_t _last_raw = 0;
    int32_t _count    = 0;
};
