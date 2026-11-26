// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"

namespace MotorDrivers {
    /*
        PI with feedforward, for closed-loop motor drivers.  The output is in percent of
        full drive and gains are per second, so neither depends on PWM resolution or loop
        rate.  The integral holds while the output is saturated and the error pushes further.
    */
    class Pid : public Configuration::Configurable {
    public:
        static constexpr float max_output = 100.0f;

        // limit narrows the clamp, and anti-windup follows it.  friction_ref only signs the
        // friction feedforward: an outer loop's correction jitters around zero at low speed,
        // and each sign flip would swing the output by twice kfs.
        float update(float setpoint, float measured, float friction_ref, float dt, float limit);

        void reset();  // call whenever the loop was not driving the output, so stale state cannot jolt it

        void group(Configuration::HandlerBase& handler) override;

    private:
        float _kp  = 0.0f;
        float _ki  = 0.0f;
        float _kff = 0.0f;
        float _kfs = 0.0f;

        float _integral = 0.0f;
    };
}
