// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Servo.h"
#include "DCMotor.h"
#include "EdgeVelocity.h"
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
        Closed-loop brushed DC motor: a DCMotor power stage, a quadrature encoder and a
        position/velocity loop.  The loop runs in its own task rather than an ISR, because
        it is float math and the ESP32 does not save FPU state on interrupt entry.
    */
    class DCServo : public Servo {
    public:
        DCServo(const char* name) : Servo(name) {}

        // Overrides for inherited methods
        void init() override;
        bool set_homing_mode(bool isHoming) override;  // protocol task: only flags the mode
        bool can_self_home() override { return _homing_stall_error != 0; }
        void set_disable(bool disable) override;  // ISR context: only flags it, the control task coasts
        void update() override;

        // Configuration handlers:
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

    protected:
        void config_message() override;

    private:
        static void control_task(void* arg);

        void resync(float setpoint, float measured);  // resume from rest without chasing an old error
        void update_filters();
        void track_motion(float setpoint_delta);
        bool rest_overshoot(float error, float& loop_error);  // true while the motor should coast
        void trip();                                          // coasts and alarms; the caller logs why

        // Returns true while the motor is held at a hard stop and must not be driven.
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
        float    _setpoint_filter_hz  = 50.0f;
        uint32_t _max_following_error = 0;
        uint32_t _position_deadband   = 0;
        uint32_t _overshoot_counts    = 0;
        uint32_t _report_ms           = 0;
        uint32_t _stall_ms            = 100;
        float    _stall_drive         = 90.0f;
        float    _stall_velocity      = 100.0f;

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
        float              _sp_alpha     = 1.0f;
        float              _vel_alpha_hz = -1.0f;  // cutoffs the alphas match, so $ changes take effect
        float              _sp_alpha_hz  = -1.0f;
        bool               _has_errors   = false;

        // Control and stall state, touched only by the control task.
        float    _offset        = 0.0f;  // encoder counts -> setpoint counts
        float    _last_setpoint = 0.0f;
        float    _last_measured = 0.0f;
        float    _setpoint_vel  = 0.0f;
        float    _measured_vel  = 0.0f;
        bool     _tripped       = false;
        bool     _trip_alarmed  = false;
        float    _report_max    = 0.0f;
        uint32_t _report_ticks  = 0;
        uint32_t _jam_ticks     = 0;

        int32_t _rest_dir = 0;      // direction of the last approach to the target
        bool    _parked   = false;  // reached the overshoot point and coasting since

        int32_t  _motion_dir  = 0;  // last setpoint step direction, 0 once stopped; signs the friction feedforward
        uint32_t _still_ticks = 0;
        uint32_t _step_gap    = 0;  // ticks between the last two steps

        EdgeVelocity _edge_vel;

        volatile bool _homing = false;

        // Stall state
        uint32_t _stall_ticks    = 0;
        bool     _stalled        = false;
        bool     _stall_positive = false;
        float    _stall_pos      = 0.0f;
    };
}
