// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Pid.h"

#include "Configuration/HandlerBase.h"

#include <algorithm>  // std::clamp

namespace MotorDrivers {
    float Pid::update(float setpoint, float measured, float friction_ref, float dt, float limit) {
        float error = setpoint - measured;

        float friction = 0.0f;
        if (friction_ref != 0.0f) {
            friction = friction_ref > 0.0f ? _kfs : -_kfs;
        }

        float base      = _kff * setpoint + friction + _kp * error;
        float integral  = _integral + _ki * error * dt;
        float candidate = base + integral;

        if ((candidate > limit && error > 0.0f) || (candidate < -limit && error < 0.0f)) {
            return std::clamp(base + _integral, -limit, limit);
        }
        _integral = integral;
        return std::clamp(candidate, -limit, limit);
    }

    void Pid::reset() {
        _integral = 0.0f;
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

        // @config kff
        // @default 0
        // @tuning per-machine
        // Feedforward gain, in percent of full drive per unit of setpoint.  It should carry
        // most of the output: about 100 over the no-load speed at full drive for a velocity loop.
        handler.item("kff", _kff, 0.0f, 1e6f);

        // @config kfs
        // @default 0
        // @tuning per-machine
        // Friction feedforward, in percent of full drive, added in the direction of the
        // planned motion.  Set it to the drive the motor needs to keep moving at low
        // speed, so the integral does not have to build that up at the start of every move.
        handler.item("kfs", _kfs, 0.0f, 100.0f);
    }
}
