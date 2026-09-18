// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "DCMotor.h"

#include "Config.h"   // Assert()
#include "Logging.h"  // log_info()

#include <algorithm>  // std::min

namespace MotorDrivers {
    void DCMotor::init() {
        _in1_pin.setAttr(Pin::Attr::PWM, _pwm_hz);
        _in2_pin.setAttr(Pin::Attr::PWM, _pwm_hz);

        // Both pins run at the same frequency, so PwmPin gives them the same
        // resolution; min() keeps a surprise there from overdriving one input
        // past the other's full scale.
        _full_duty = std::min(_in1_pin.maxDuty(), _in2_pin.maxDuty());
        _max_duty  = _full_duty * _max_duty_percent / 100;

        // New PwmPins start at zero duty, so the cached values already match.
        _in1_duty = 0;
        _in2_duty = 0;
        _duty     = 0;

        config_message();
    }

    void DCMotor::set_disable(bool disable) {
        if (disable) {
            coast();
        }
    }

    void DCMotor::set(int32_t duty) {
        const int32_t limit = int32_t(_max_duty);
        if (duty > limit) {
            duty = limit;
        } else if (duty < -limit) {
            duty = -limit;
        }
        _duty = duty;

        if (_invert_direction) {
            duty = -duty;
        }

        if (duty >= 0) {
            write(uint32_t(duty), 0);
        } else {
            write(0, uint32_t(-duty));
        }
    }

    void DCMotor::brake() {
        _duty = 0;
        write(_full_duty, _full_duty);
    }

    void DCMotor::coast() {
        _duty = 0;
        write(0, 0);
    }

    void DCMotor::write(uint32_t in1, uint32_t in2) {
        // Lower an input before raising the other one.  Raising first would put
        // both inputs high for the duration of the two register writes, which
        // the A4950 decodes as brake - a torque spike in the wrong direction on
        // every direction reversal.
        if (in1 < _in1_duty) {
            _in1_pin.setDuty(in1);
            _in1_duty = in1;
        }
        if (in2 < _in2_duty) {
            _in2_pin.setDuty(in2);
            _in2_duty = in2;
        }
        if (in1 != _in1_duty) {
            _in1_pin.setDuty(in1);
            _in1_duty = in1;
        }
        if (in2 != _in2_duty) {
            _in2_pin.setDuty(in2);
            _in2_duty = in2;
        }
    }

    void DCMotor::validate() {
        Assert(!_in1_pin.undefined(), "in1_pin should be configured.");
        Assert(!_in2_pin.undefined(), "in2_pin should be configured.");

        // Sign-magnitude drive PWMs whichever input matches the commanded
        // direction, so both of them have to be PWM-capable - an I2SO or
        // extender pin will not do.
        Assert(_in1_pin.capabilities().has(Pin::Capabilities::PWM), "in1_pin %s cannot do PWM", _in1_pin.name());
        Assert(_in2_pin.capabilities().has(Pin::Capabilities::PWM), "in2_pin %s cannot do PWM", _in2_pin.name());

        Assert(_in1_pin.index() != _in2_pin.index(), "in1_pin and in2_pin must be different pins");
    }

    void DCMotor::group(Configuration::HandlerBase& handler) {
        // @config in1_pin
        // @default NO_PIN
        // @pin_attributes pwm
        // H-bridge input 1, e.g. IN1 on an A4950.  Carries the PWM when the motor is
        // driven in the positive direction, and is held low otherwise.
        handler.item("in1_pin", _in1_pin);

        // @config in2_pin
        // @default NO_PIN
        // @pin_attributes pwm
        // H-bridge input 2, e.g. IN2 on an A4950.  Carries the PWM when the motor is
        // driven in the negative direction, and is held low otherwise.
        handler.item("in2_pin", _in2_pin);

        // @config pwm_hz
        // @default 20000
        // Switching frequency for both inputs.  20 kHz and up keeps the motor out of
        // the audible range; the cost is duty resolution, since the LEDC bit depth is
        // derived from this frequency -- 20 kHz leaves 11 bits, 40 kHz only 10.
        handler.item("pwm_hz", _pwm_hz, 100, 100000);

        // @config invert_direction
        // @default false
        // Swaps the roles of in1_pin and in2_pin, for a motor wired backwards relative
        // to the axis direction.  Prefer this over rewiring or negating steps_per_mm.
        handler.item("invert_direction", _invert_direction);

        // @config max_duty_percent
        // @default 100
        // Ceiling on the duty set() will apply, as a percentage of full scale.  Use it
        // to cap average motor voltage when the supply is higher than the motor is
        // rated for.  brake() ignores it, since a partial brake is not a brake.
        handler.item("max_duty_percent", _max_duty_percent, 1, 100);
    }

    void DCMotor::config_message() {
        log_info("    DC motor IN1:" << _in1_pin.name() << " IN2:" << _in2_pin.name() << " PWM:" << _pwm_hz << "Hz duty:" << _max_duty
                                     << "/" << _full_duty << (_invert_direction ? " inverted" : ""));
    }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<DCMotor> registration("dcmotor");
    }
}
