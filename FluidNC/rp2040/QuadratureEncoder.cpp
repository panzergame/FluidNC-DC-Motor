// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
  No quadrature decoder on RP2040 yet.  A PIO program would do it, but until
  then the encoder reports invalid so that drivers depending on it refuse to
  run rather than drive a motor with no feedback.
*/

#include "Config.h"
#include "Driver/QuadratureEncoder.h"

QuadratureEncoder::QuadratureEncoder(pinnum_t a, pinnum_t b, uint32_t filter_ns) {
    log_error("Quadrature encoder is not supported on this platform");
}

int32_t QuadratureEncoder::read() {
    return 0;
}
