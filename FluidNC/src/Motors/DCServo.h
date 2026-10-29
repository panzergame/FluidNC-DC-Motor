// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Servo.h"
#include "DCMotor.h"
#include "Pid.h"
#include "Pin.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class QuadratureEncoder;

namespace MotorDrivers {
    /*
        Closed-loop brushed DC motor: a DCMotor power stage, a quadrature
        encoder, and a cascaded position/velocity loop.

        The planner and step segment generator already enforce the axis
        acceleration and max_rate, and the step ISR advances the axis step
        count along that profile whether or not a stepper is attached.  That
        count is the position setpoint here, so this driver does no motion
        profiling of its own - only tracking:

            velocity command = setpoint velocity + position_kp * position error
            drive            = velocity_pid(velocity command, measured velocity)

        The loop runs in its own task rather than an ISR, because it is float
        math and the ESP32 does not save FPU state on interrupt entry.

        Homing has to come from limit switches; there is no index pulse.
    */
    class DCServo : public Servo {
    public:
        DCServo(const char* name) : Servo(name) {}

        void init() override;

        bool set_homing_mode(bool isHoming) override { return false; }
        bool can_self_home() override { return false; }

        // Called from ISR context by Axes, so it only flags the request; the
        // control task does the coasting.
        void set_disable(bool disable) override;

        // One control tick.
        void update() override;

        // Configuration handlers:
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

    protected:
        void config_message() override;

    private:
        static void control_task(void* arg);

        // Takes the current encoder reading as being where the setpoint is,
        // so control resumes from rest without chasing an old error.
        void resync(float setpoint, float measured);

        void trip(float error);

        DCMotor* _dc  = nullptr;
        Pid*     _pid = nullptr;

        Pin      _encoder_a_pin;
        Pin      _encoder_b_pin;
        uint32_t _encoder_filter_ns = 1000;
        bool     _encoder_invert    = false;

        float    _counts_per_step     = 1.0f;
        uint32_t _update_ms           = 1;
        float    _position_kp         = 0.0f;
        float    _velocity_filter_hz  = 100.0f;
        uint32_t _max_following_error = 0;
        uint32_t _position_deadband   = 0;
        uint32_t _report_ms           = 0;

        QuadratureEncoder* _encoder      = nullptr;
        TickType_t         _update_ticks = 1;
        float              _dt           = 0.001f;
        float              _vel_alpha    = 1.0f;
        bool               _has_errors   = false;

        // Control state, touched only by the control task.
        float    _offset        = 0.0f;  // encoder counts -> setpoint counts
        float    _last_setpoint = 0.0f;
        float    _last_measured = 0.0f;
        float    _setpoint_vel  = 0.0f;
        float    _measured_vel  = 0.0f;
        bool     _tripped       = false;
        bool     _trip_alarmed  = false;
        float    _report_max    = 0.0f;
        uint32_t _report_ticks  = 0;
        float    _last_drive    = 0.0f;
    };
}
