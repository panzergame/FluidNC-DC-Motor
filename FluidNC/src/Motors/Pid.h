// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"

namespace MotorDrivers {
    /*
        PID with a feedforward term, for closed-loop motor drivers.

        The output is in percent of full drive, clamped to +/-100, so gains do
        not depend on the PWM resolution.  dt is passed on every update, so
        gains are per second and do not change with the loop rate.

        D acts on the measurement rather than the error, so a step in the
        setpoint does not kick the output.  Windup is prevented by conditional
        integration: the integral holds while the output is saturated and the
        error pushes further into saturation.
    */
    class Pid : public Configuration::Configurable {
    public:
        float update(float setpoint, float measured, float dt);

        // Clears the integral and derivative history.  Call it whenever the
        // loop has not been driving the output, so that stale state does not
        // jolt the motor when control resumes.
        void reset();

        void group(Configuration::HandlerBase& handler) override;

    private:
        float _kp          = 0.0f;
        float _ki          = 0.0f;
        float _kd          = 0.0f;
        float _kff         = 0.0f;
        float _d_filter_hz = 0.0f;

        float _integral      = 0.0f;
        float _d_term        = 0.0f;
        float _prev_measured = 0.0f;
        bool  _primed        = false;
    };
}
