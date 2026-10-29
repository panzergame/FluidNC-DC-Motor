// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    Example YAML

      dc_servo:
        encoder_a_pin: gpio.34
        encoder_b_pin: gpio.35
        counts_per_step: 1
        position_kp: 20
        max_following_error: 200
        homing_stall_error: 100
        homing_stall_velocity: 50
        homing_stall_ms: 50
        homing_max_drive: 30
        dc_motor:
          in1_pin: gpio.32
          in2_pin: gpio.33
          pwm_hz: 20000
        velocity_pid:
          kp: 0.005
          ki: 0.05
          kff: 0.01
*/

#include "DCServo.h"

#include "Driver/QuadratureEncoder.h"
#include "Machine/Homing.h"
#include "Machine/MachineConfig.h"
#include "Machine/Motor.h"
#include "MotionControl.h"  // mc_critical()
#include "Protocol.h"       // protocol_send_event()
#include "Event.h"
#include "System.h"  // get_axis_steps(), state_is()

#include <cmath>  // std::fabs, std::exp

namespace MotorDrivers {
    // Below the WiFi and timer tasks so they keep their latency, above the
    // protocol and polling tasks so that serial traffic does not stretch the
    // control period.
    static constexpr UBaseType_t control_task_priority = 10;

    // Stopping the stepper has to happen on the protocol task, not this one.
    static void do_following_error() {
        mc_critical(ExecAlarm::HardStop);
    }
    static const NoArgEvent followingErrorEvent { do_following_error };

    void DCServo::init() {
        _axis  = axis_index();
        _motor = Axes::_axis[_axis]->_motors[dual_axis_index()];

        _dc->init();

        _encoder_a_pin.setAttr(Pin::Attr::Input);
        _encoder_b_pin.setAttr(Pin::Attr::Input);
        _encoder = new QuadratureEncoder(_encoder_a_pin.getNative(Pin::Capabilities::Input | Pin::Capabilities::Native),
                                         _encoder_b_pin.getNative(Pin::Capabilities::Input | Pin::Capabilities::Native),
                                         _encoder_filter_ns);
        if (!_encoder->valid()) {
            log_config_error("    " << name() << " disabled: no encoder");
            _has_errors = true;
            return;
        }

        _update_ticks = pdMS_TO_TICKS(_update_ms);
        if (_update_ticks == 0) {
            _update_ticks = 1;
        }
        _dt        = float(_update_ticks) / float(configTICK_RATE_HZ);
        _vel_alpha = 1.0f - std::exp(-2.0f * float(M_PI) * _velocity_filter_hz * _dt);

        config_message();

        xTaskCreateAffinitySet(control_task, "dc_servo", 4096, this, control_task_priority, (1 << SUPPORT_TASK_CORE), nullptr);
    }

    void DCServo::control_task(void* arg) {
        auto       servo = static_cast<DCServo*>(arg);
        TickType_t last  = xTaskGetTickCount();
        while (true) {
            vTaskDelayUntil(&last, servo->_update_ticks);
            servo->update();
        }
    }

    void IRAM_ATTR DCServo::set_disable(bool disable) {
        _disabled = disable;
    }

    bool DCServo::set_homing_mode(bool isHoming) {
        _homing = isHoming;
        return true;
    }

    void DCServo::resync(float setpoint, float measured) {
        _offset        = setpoint - measured;
        _last_setpoint = setpoint;
        _last_measured = measured;
        _setpoint_vel  = 0.0f;
        _measured_vel  = 0.0f;
        _pid->reset();
    }

    void DCServo::trip(float error) {
        _tripped      = true;
        _trip_alarmed = false;
        _dc->coast();
        log_error(name() << " " << axisName() << " following error " << error << " counts exceeds " << _max_following_error);
        protocol_send_event(&followingErrorEvent);
    }

    bool DCServo::stall_check(float setpoint, float measured, float error, bool approaching) {
        if (_stalled) {
            if (approaching) {
                // The setpoint keeps advancing until the limit event stops
                // the planner; tracking it would only push into the stop.
                resync(setpoint, measured);
                _dc->set(0);
                _last_drive = 0.0f;
                return true;
            }
            if (std::fabs(measured - _stall_pos) > float(_homing_release_counts)) {
                release_stall();
            }
            return false;
        }

        if (!approaching || !_homing_stall_error) {
            _stall_ticks = 0;
            return false;
        }

        // Error alone also grows while accelerating, and low velocity alone
        // also happens between steps at slow feeds; only both together, held
        // for a while, mean the axis is pushing against something.
        if (std::fabs(error) > float(_homing_stall_error) && std::fabs(_measured_vel) < _homing_stall_velocity) {
            ++_stall_ticks;
        } else {
            _stall_ticks = 0;
        }
        if (float(_stall_ticks) * _dt * 1000.0f < float(_homing_stall_ms)) {
            return false;
        }

        _stall_ticks    = 0;
        _stalled        = true;
        _stall_positive = error > 0.0f;
        _stall_pos      = measured;
        log_debug(name() << " " << axisName() << " stalled at " << (measured + _offset) << " error " << error);
        resync(setpoint, measured);
        _dc->set(0);
        _last_drive = 0.0f;
        _motor->softLimit(_stall_positive, true);
        return true;
    }

    void DCServo::release_stall() {
        _stalled = false;
        _motor->softLimit(_stall_positive, false);
    }

    void DCServo::update() {
        const int32_t raw      = _encoder->read();
        const float   measured = float(_encoder_invert ? -raw : raw);
        const float   setpoint = float(get_axis_steps(_axis)) * _counts_per_step;

        // Ahead of the disabled path, so that a homing failure that also
        // disables the motor still clears the soft limit.
        if (_stalled && !_homing) {
            release_stall();
        }

        if (_tripped) {
            // Hold off until the alarm has actually been raised and then
            // cleared; checking only for "not in alarm" would release the
            // trip before the event has even been processed.
            if (state_is(State::Alarm)) {
                _trip_alarmed = true;
            } else if (_trip_alarmed) {
                _tripped = false;
            }
        }

        if (_disabled || _tripped) {
            if (_last_drive != 0.0f) {
                _dc->coast();
                _last_drive = 0.0f;
            }
            resync(setpoint, measured);
            return;
        }

        // The setpoint can only move as fast as max_rate allows, so a larger
        // jump means the position was redefined - homing, or a reset after an
        // alarm - rather than commanded.  Chasing it would be a lurch.
        auto        axis     = Axes::_axis[_axis];
        const float max_step = axis->_maxRate / 60.0f * axis->_stepsPerMm * _counts_per_step * _dt;
        if (std::fabs(setpoint - _last_setpoint) > 4.0f * max_step + 2.0f) {
            resync(setpoint, measured);
            return;
        }

        _setpoint_vel += ((setpoint - _last_setpoint) / _dt - _setpoint_vel) * _vel_alpha;
        _measured_vel += ((measured - _last_measured) / _dt - _measured_vel) * _vel_alpha;
        _last_setpoint = setpoint;
        _last_measured = measured;

        const float error       = setpoint - (measured + _offset);
        const bool  approaching = _homing && Machine::Homing::approach();
        // Approaching a hard stop is meant to build up error.
        if (!(approaching && _homing_stall_error) && _max_following_error && std::fabs(error) > float(_max_following_error)) {
            trip(error);
            _last_drive = 0.0f;
            return;
        }

        if (stall_check(setpoint, measured, error, approaching)) {
            return;
        }

        // Stiction stops the motor from correcting the last count or two, so
        // at rest the integral would only wind up into a stalled-motor drive
        // that heats the motor and pushes along with the next disturbance.
        // Idle rather than a zero setpoint velocity, because at low feed rates
        // the setpoint sits still for many ticks between steps mid-move.
        float velocity_cmd = 0.0f;
        float drive        = 0.0f;
        if (_position_deadband && state_is(State::Idle) && std::fabs(error) <= float(_position_deadband)) {
            _pid->reset();
        } else {
            velocity_cmd = _setpoint_vel + _position_kp * error;
            drive        = _pid->update(velocity_cmd, _measured_vel, _dt, _homing ? _homing_max_drive : Pid::max_output);
        }
        _dc->set(int32_t(drive * float(_dc->maxDuty()) / 100.0f));
        _last_drive = drive;

        if (_report_ms) {
            if (std::fabs(error) > _report_max) {
                _report_max = std::fabs(error);
            }
            _report_ticks += _update_ticks;
            if (_report_ticks >= pdMS_TO_TICKS(_report_ms)) {
                log_info(axisName() << " sp:" << setpoint << " pos:" << (measured + _offset) << " err_max:" << _report_max
                                    << " vel:" << _measured_vel << "/" << velocity_cmd << " drive:" << drive << "%");
                _report_ticks = 0;
                _report_max   = 0.0f;
            }
        }
    }

    void DCServo::validate() {
        Assert(_dc != nullptr, "dc_motor section is required");
        Assert(_pid != nullptr, "velocity_pid section is required");
        Assert(!_encoder_a_pin.undefined(), "encoder_a_pin should be configured.");
        Assert(!_encoder_b_pin.undefined(), "encoder_b_pin should be configured.");
        Assert(_encoder_a_pin.index() != _encoder_b_pin.index(), "encoder_a_pin and encoder_b_pin must be different pins");
    }

    void DCServo::group(Configuration::HandlerBase& handler) {
        // @config encoder_a_pin
        // @default NO_PIN
        // @pin_attributes input
        // Encoder channel A.  Must be a native GPIO, since it feeds the pulse counter.
        handler.item("encoder_a_pin", _encoder_a_pin);

        // @config encoder_b_pin
        // @default NO_PIN
        // @pin_attributes input
        // Encoder channel B.  Must be a native GPIO, since it feeds the pulse counter.
        handler.item("encoder_b_pin", _encoder_b_pin);

        // @config encoder_filter_ns
        // @default 1000
        // @tuning typical
        // Encoder pulses shorter than this are treated as noise.  The ESP32 caps it at
        // about 12.7us.  0 disables the filter.
        handler.item("encoder_filter_ns", _encoder_filter_ns, 0, 100000);

        // @config encoder_invert
        // @default false
        // Reverses the encoder count direction.  It must agree with the motor direction:
        // if the axis runs away at full drive as soon as it is enabled, flip this.
        handler.item("encoder_invert", _encoder_invert);

        // @config counts_per_step
        // @default 1
        // @tuning per-machine
        // Encoder counts (after x4 decoding) per axis step.  Leave at 1 and set the
        // axis steps_per_mm in encoder counts unless the two need to differ.
        handler.item("counts_per_step", _counts_per_step, 0.001f, 1000.0f);

        // @config update_ms
        // @default 1
        // @tuning typical
        // Control loop period.  Rounded to whole FreeRTOS ticks.
        handler.item("update_ms", _update_ms, 1, 100);

        // @config position_kp
        // @default 0
        // @tuning per-machine
        // Outer position loop gain, in counts per second of velocity command per count of
        // position error.  The setpoint velocity is fed forward, so this only has to
        // correct the residual error.
        handler.item("position_kp", _position_kp, 0.0f, 1e5f);

        // @config velocity_filter_hz
        // @default 100
        // @tuning typical
        // Low-pass cutoff on both the measured and setpoint velocities, which are otherwise
        // quantized to whole counts per control period.
        handler.item("velocity_filter_hz", _velocity_filter_hz, 1.0f, 10000.0f);

        // @config max_following_error
        // @default 0
        // @tuning per-machine
        // Position error, in encoder counts, beyond which the motor coasts and the machine
        // alarms.  0 disables the check.
        handler.item("max_following_error", _max_following_error, 0, 1000000);

        // @config position_deadband
        // @default 0
        // @tuning per-machine
        // While the machine is idle and the position error is within this many encoder
        // counts, the motor coasts and the velocity PID is reset.  Set it just above the
        // error stiction leaves at rest, so the loop does not hold drive on a stalled motor.
        // 0 disables it.
        handler.item("position_deadband", _position_deadband, 0, 10000);

        // @config homing_stall_error
        // @default 0
        // @tuning per-machine
        // Homes against a hard stop instead of a switch: during a homing approach, a
        // following error above this many encoder counts, with the axis below
        // homing_stall_velocity for homing_stall_ms, raises this motor's limit.  It must
        // be above the error a normal approach builds up while accelerating.  0 disables
        // stall homing, so the axis needs limit switches to home.
        handler.item("homing_stall_error", _homing_stall_error, 0, 1000000);

        // @config homing_stall_velocity
        // @default 100
        // @tuning per-machine
        // Measured speed, in encoder counts per second, below which the axis counts as
        // stopped for stall detection.
        handler.item("homing_stall_velocity", _homing_stall_velocity, 0.0f, 1e7f);

        // @config homing_stall_ms
        // @default 50
        // @tuning per-machine
        // How long the stall condition has to hold before the limit is raised.
        handler.item("homing_stall_ms", _homing_stall_ms, 1, 10000);

        // @config homing_release_counts
        // @default 10
        // @tuning per-machine
        // After a stall, the soft limit clears once the axis has backed this many encoder
        // counts off the stop.  Must be less than the motor's pulloff_mm in counts, or
        // homing fails with the limit still engaged.
        handler.item("homing_release_counts", _homing_release_counts, 1, 1000000);

        // @config homing_max_drive
        // @default 100
        // @tuning per-machine
        // Caps the drive, in percent, for the whole homing cycle, which bounds the force
        // the axis pushes into a hard stop with.  Leave enough to overcome friction at
        // the homing feed rate.
        handler.item("homing_max_drive", _homing_max_drive, 1.0f, 100.0f);

        // @config report_ms
        // @default 0
        // Interval for logging setpoint, position, peak following error and drive while
        // tuning.  0 disables it.
        handler.item("report_ms", _report_ms, 0, 60000);

        handler.section("dc_motor", _dc, "dc_motor");
        handler.section("velocity_pid", _pid);

        Servo::group(handler);
    }

    void DCServo::config_message() {
        log_info("    " << name() << " Encoder A:" << _encoder_a_pin.name() << " B:" << _encoder_b_pin.name() << " period:" << _update_ms
                        << "ms" << (_encoder_invert ? " inverted" : ""));
    }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<DCServo> registration("dc_servo");
    }
}
