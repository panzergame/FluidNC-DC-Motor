// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "Driver/QuadratureEncoder.h"
#include "Pin.h"

namespace MotorDrivers {
    // Configuration and ownership of a QuadratureEncoder, with the count direction applied.
    class Encoder : public Configuration::Configurable {
    public:
        bool init();  // false if no counter could be allocated

        EncoderSample sample();

        bool     has_timing() const { return _encoder->has_timing(); }
        uint32_t timer_hz() const { return _encoder->timer_hz(); }
        float    counts_per_step() const { return _counts_per_step; }
        bool     inverted() const { return _invert; }

        const Pin& a_pin() const { return _a_pin; }
        const Pin& b_pin() const { return _b_pin; }

        // Configuration handlers:
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

    private:
        Pin      _a_pin;
        Pin      _b_pin;
        uint32_t _filter_ns       = 1000;
        bool     _invert          = false;
        float    _counts_per_step = 1.0f;

        QuadratureEncoder* _encoder = nullptr;
    };
}
