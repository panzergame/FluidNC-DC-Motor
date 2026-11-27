// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "Pin.h"

namespace MotorDrivers {
    /*
        Power stage for a brushed DC motor behind a two-input H-bridge such as the
        Allegro A4950 (IN1/IN2).  Sign-magnitude drive with slow decay: the input for the
        requested direction is held high and the other carries the inverted PWM, so the
        off-time is spent in brake rather than coast.  Motor current stays continuous,
        duty maps nearly linearly onto torque down to zero, and back-EMF damps the motor
        at every duty.  Lock-antiphase would be as linear but pushes ripple current
        through the motor even at standstill.
    */
    class DCMotor : public Configuration::Configurable {
    public:
        void init();  // must run before set()

        // Signed duty, clamped to +/- maxDuty(); positive increases axis position.  Zero
        // brakes.
        void set(int32_t duty);

        void brake();  // both inputs high - motor terminals shorted
        void coast();  // both inputs low  - motor terminals open

        uint32_t maxDuty() const { return _max_duty; }  // full scale reduced by max_duty_percent

        // Configuration handlers:
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

    private:
        void config_message();
        void write(uint32_t in1, uint32_t in2);

        Pin      _in1_pin;
        Pin      _in2_pin;
        uint32_t _pwm_hz           = 20000;
        bool     _invert_direction = false;
        uint32_t _max_duty_percent = 100;

        uint32_t _full_duty = 0;  // PWM full scale, known after init(); brake() uses it
        uint32_t _max_duty  = 0;

        // Last duty written to each input, so write() can skip redundant register writes.
        uint32_t _in1_duty = 0;
        uint32_t _in2_duty = 0;
    };
}
