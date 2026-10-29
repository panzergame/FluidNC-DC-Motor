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

namespace Machine {
    class Motor;
}

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

        There is no index pulse, so homing comes either from limit switches
        or, with homing_stall_error set, from driving into a hard stop: when
        the axis stops following the setpoint during a homing approach, the
        motor's limit is raised in software as if a switch had fired.
    */
    class DCServo : public Servo {
    public:
        DCServo(const char* name) : Servo(name) {}

        void init() override;

        // Called from the protocol task, so it only flags the mode; the
        // control task acts on it.
        bool set_homing_mode(bool isHoming) override;
        bool can_self_home() override { return _homing_stall_error != 0; }

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

        // Detects the hard stop while approaching, and releases the soft
        // limit once the axis has backed off it.  Returns true if the motor
        // is being held at the stop, in which case it must not be driven.
        bool stall_check(float setpoint, float measured, float error, bool approaching);
        void release_stall();

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

        uint32_t _homing_stall_error    = 0;
        float    _homing_stall_velocity = 100.0f;
        uint32_t _homing_stall_ms       = 50;
        uint32_t _homing_release_counts = 10;
        float    _homing_max_drive      = 100.0f;

        QuadratureEncoder* _encoder      = nullptr;
        Machine::Motor*    _motor        = nullptr;
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

        volatile bool _homing = false;

        // Stall state, touched only by the control task.
        uint32_t _stall_ticks    = 0;
        bool     _stalled        = false;
        bool     _stall_positive = false;
        float    _stall_pos      = 0.0f;
    };
}
