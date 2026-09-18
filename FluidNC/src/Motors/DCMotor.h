// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "MotorDriver.h"
#include "Pin.h"

namespace MotorDrivers {
    /*
        Output stage for a brushed DC motor behind a two-input H-bridge such as
        the Allegro A4950 (IN1/IN2).

        Sign-magnitude drive: the input for the requested direction carries the
        PWM while the other is held low.  That makes duty map directly onto
        average motor voltage, and makes zero duty a coast rather than a brake.
        The alternative, lock-antiphase, is more linear near zero but pushes
        ripple current through the motor even at standstill.

        This is only the power stage - it has no encoder, no PID and no notion
        of position, so an axis configured with it will accept a move and not
        go anywhere.  It registers as a motor driver anyway so that the pins and
        the duty mapping can be exercised from a config file and the $M*
        commands before any of the closed-loop machinery exists.  Once the
        closed-loop driver lands, that driver owns one of these as a
        handler.section("dc_motor", ...) member and this registration goes away.
    */
    class DCMotor : public MotorDriver {
    public:
        DCMotor(const char* name) : MotorDriver(name) {}

        // init() resolves the duty scale, so it must run before set().  It
        // cannot happen in the constructor because the duty resolution depends
        // on pwm_hz, which is not known until the config has been parsed.
        void init() override;

        // Nothing here can find a reference position, and there is no feedback
        // to home against, so homing has to come from limit switches.
        bool set_homing_mode(bool isHoming) override { return false; }
        bool can_self_home() override { return false; }

        // Idle release coasts rather than brakes; a braked H-bridge still
        // dissipates whatever the axis backdrives into it.
        void set_disable(bool disable) override;

        // Signed duty, clamped to +/- maxDuty().  Positive drives the direction
        // that increases axis position, subject to invert_direction.  Zero
        // coasts; use brake() for the other kind of stop.
        void set(int32_t duty);

        void brake();  // both inputs high - motor terminals shorted
        void coast();  // both inputs low  - motor terminals open

        // Largest magnitude set() will act on, i.e. full scale reduced by
        // max_duty_percent.
        uint32_t maxDuty() const { return _max_duty; }

        // Last value passed to set(), for diagnostics.  brake() and coast()
        // reset it to zero, since neither is expressible as a signed duty.
        int32_t duty() const { return _duty; }

        // Configuration handlers:
        void validate() override;
        void group(Configuration::HandlerBase& handler) override;

    protected:
        void config_message() override;

    private:
        void write(uint32_t in1, uint32_t in2);

        Pin      _in1_pin;
        Pin      _in2_pin;
        uint32_t _pwm_hz           = 20000;
        bool     _invert_direction = false;
        uint32_t _max_duty_percent = 100;

        // Full scale of the PWM pins, resolved in init() because PwmPin derives
        // the LEDC bit depth from _pwm_hz.  brake() uses this rather than
        // _max_duty: a partial brake is not a brake.
        uint32_t _full_duty = 0;
        uint32_t _max_duty  = 0;

        // Last duty written to each input, so write() can skip redundant
        // register writes the way PwmServo::_write_pwm() does.
        uint32_t _in1_duty = 0;
        uint32_t _in2_duty = 0;

        int32_t _duty = 0;
    };
}
