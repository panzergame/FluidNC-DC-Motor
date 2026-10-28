// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Pid.h"

#include "Configuration/HandlerBase.h"

#include <algorithm>  // std::clamp
#include <cmath>      // M_PI

namespace MotorDrivers {
    static constexpr float max_output = 100.0f;

    float Pid::update(float setpoint, float measured, float dt) {
        const float error = setpoint - measured;

        // The first sample has no previous measurement to differentiate
        // against; differentiating against zero would be a spike.
        if (_primed) {
            float d = -_kd * (measured - _prev_measured) / dt;
            if (_d_filter_hz > 0.0f) {
                const float wdt = 2.0f * float(M_PI) * _d_filter_hz * dt;
                _d_term += (d - _d_term) * wdt / (1.0f + wdt);
            } else {
                _d_term = d;
            }
        }
        _prev_measured = measured;
        _primed        = true;

        const float base      = _kff * setpoint + _kp * error + _d_term;
        const float integral  = _integral + _ki * error * dt;
        const float candidate = base + integral;

        if ((candidate > max_output && error > 0.0f) || (candidate < -max_output && error < 0.0f)) {
            return std::clamp(base + _integral, -max_output, max_output);
        }
        _integral = integral;
        return std::clamp(candidate, -max_output, max_output);
    }

    void Pid::reset() {
        _integral = 0.0f;
        _d_term   = 0.0f;
        _primed   = false;
    }

    void Pid::group(Configuration::HandlerBase& handler) {
        // @config kp
        // @default 0
        // @tuning per-machine
        // Proportional gain, in percent of full drive per unit of error.
        handler.item("kp", _kp, 0.0f, 1e6f);

        // @config ki
        // @default 0
        // @tuning per-machine
        // Integral gain, in percent of full drive per unit of error per second.
        handler.item("ki", _ki, 0.0f, 1e6f);

        // @config kd
        // @default 0
        // @tuning per-machine
        // Derivative gain, in percent of full drive per unit of measurement change per second.
        // Acts on the measurement only, so setpoint steps do not kick the output.
        handler.item("kd", _kd, 0.0f, 1e6f);

        // @config kff
        // @default 0
        // @tuning per-machine
        // Feedforward gain, in percent of full drive per unit of setpoint.  For a velocity
        // loop this is roughly 100 divided by the no-load speed at full drive, and should
        // carry most of the output so that kp and ki only correct the remainder.
        handler.item("kff", _kff, 0.0f, 1e6f);

        // @config d_filter_hz
        // @default 0
        // @tuning typical
        // Cutoff of a first-order low-pass on the derivative term.  0 disables the filter.
        handler.item("d_filter_hz", _d_filter_hz, 0.0f, 1e5f);
    }
}
