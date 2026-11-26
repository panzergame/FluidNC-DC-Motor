// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Encoder.h"

#include "Configuration/HandlerBase.h"

namespace MotorDrivers {
    bool Encoder::init() {
        _a_pin.setAttr(Pin::Attr::Input);
        _b_pin.setAttr(Pin::Attr::Input);
        _encoder = new QuadratureEncoder(_a_pin.getNative(Pin::Capabilities::Input | Pin::Capabilities::Native),
                                         _b_pin.getNative(Pin::Capabilities::Input | Pin::Capabilities::Native),
                                         _filter_ns);
        return _encoder->valid();
    }

    EncoderSample Encoder::sample() {
        EncoderSample sample = _encoder->sample();
        if (_invert) {
            sample.count = -sample.count;
        }
        return sample;
    }

    void Encoder::validate() {
        Assert(!_a_pin.undefined(), "a_pin should be configured.");
        Assert(!_b_pin.undefined(), "b_pin should be configured.");
        Assert(_a_pin.index() != _b_pin.index(), "a_pin and b_pin must be different pins");
    }

    void Encoder::group(Configuration::HandlerBase& handler) {
        // @config a_pin
        // @default NO_PIN
        // @pin_attributes input
        // Encoder channel A.  Must be a native GPIO, since it feeds the pulse counter.
        handler.item("a_pin", _a_pin);

        // @config b_pin
        // @default NO_PIN
        // @pin_attributes input
        // Encoder channel B.  Must be a native GPIO, since it feeds the pulse counter.
        handler.item("b_pin", _b_pin);

        // @config filter_ns
        // @default 1000
        // @tuning typical
        // Encoder pulses shorter than this are treated as noise.  The ESP32 caps it at
        // about 12.7us.  0 disables the filter.
        handler.item("filter_ns", _filter_ns, 0, 100000);

        // @config invert
        // @default false
        // Reverses the encoder count direction.  It must agree with the motor direction:
        // if the axis runs away at full drive as soon as it is enabled, flip this.
        handler.item("invert", _invert);

        // @config counts_per_step
        // @default 1
        // @tuning per-machine
        // Encoder counts (after x4 decoding) per axis step.  Leave at 1 and set the
        // axis steps_per_mm in encoder counts unless the two need to differ.
        handler.item("counts_per_step", _counts_per_step, 0.001f, 1000.0f);
    }
}
