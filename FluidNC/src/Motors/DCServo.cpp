// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    A brushed DC motor with a quadrature encoder, run as a servo.

    The planner and step segment generator already enforce acceleration and max_rate,
    and the axis step count follows that profile whether or not a stepper is attached.
    That count is the position setpoint, so this driver does no profiling, only tracking:

        velocity command = setpoint velocity + position_kp * position error
        drive            = velocity_pid(velocity command, measured velocity)

    There is no index pulse, so homing needs limit switches, or homing_stall_error to
    home against a hard stop: when the axis stops following the setpoint during a homing
    approach, the motor's limit is raised in software as if a switch had fired.

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

#include <algorithm>  // std::min
#include <cmath>      // std::fabs, std::exp

namespace MotorDrivers {
    // Below the WiFi and timer tasks so they keep their latency, above the
    // protocol and polling tasks so that serial traffic does not stretch the
    // control period.
    static constexpr UBaseType_t control_task_priority = 10;

    // Longest the setpoint may sit still between steps and still count as
    // moving.  Slower feeds than this spacing allows are as good as stopped.
    static constexpr float max_step_gap_s = 0.25f;

    static float lowpass_alpha(float hz, float dt) {
        return 1.0f - std::exp(-2.0f * float(M_PI) * hz * dt);
    }

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
        _dt = float(_update_ticks) / float(configTICK_RATE_HZ);
        update_filters();

        config_message();

        xTaskCreateAffinitySet(control_task,              // task
                               "dc_servo",                // name for task
                               4096,                      // size of task stack
                               this,                      // parameters
                               control_task_priority,     // priority
                               (1 << SUPPORT_TASK_CORE),  // affinity mask
                               nullptr                    // task handle
        );
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

    void DCServo::update_filters() {
        if (_velocity_filter_hz != _vel_alpha_hz) {
            _vel_alpha    = lowpass_alpha(_velocity_filter_hz, _dt);
            _vel_alpha_hz = _velocity_filter_hz;
        }
        if (_setpoint_filter_hz != _sp_alpha_hz) {
            _sp_alpha    = lowpass_alpha(_setpoint_filter_hz, _dt);
            _sp_alpha_hz = _setpoint_filter_hz;
        }
    }

    void DCServo::resync(float setpoint, float measured) {
        _offset        = setpoint - measured;
        _last_setpoint = setpoint;
        _last_measured = measured;
        _setpoint_vel  = 0.0f;
        _measured_vel  = 0.0f;
        _jam_ticks     = 0;
        _rest_dir      = 0;
        _parked        = false;
        _motion_dir    = 0;
        _still_ticks   = 0;
        _edge_vel.reset();
        _pid->reset();
    }

    // The filtered setpoint velocity dips between slow steps and decays slowly after a move,
    // so the step spacing decides how long a pause still belongs to the motion.
    void DCServo::track_motion(float setpoint_delta) {
        uint32_t max_gap = uint32_t(max_step_gap_s / _dt);
        if (setpoint_delta != 0.0f) {
            int32_t dir = setpoint_delta > 0.0f ? 1 : -1;
            // From rest the spacing is unknown until the second step.
            _step_gap    = dir == _motion_dir ? _still_ticks + 1 : max_gap;
            _motion_dir  = dir;
            _still_ticks = 0;
            return;
        }
        if (_motion_dir == 0) {
            return;
        }
        ++_still_ticks;
        uint32_t hold = std::min(2 * _step_gap, max_gap);  // twice the spacing rides out step jitter
        if (_still_ticks > hold) {
            _motion_dir = 0;
        }
    }

    void DCServo::trip() {
        _tripped      = true;
        _trip_alarmed = false;
        _dc->coast();
        protocol_send_event(&followingErrorEvent);
    }

    bool DCServo::stall_check(float setpoint, float measured, float error, bool approaching) {
        if (_stalled) {
            if (approaching) {
                // The setpoint keeps advancing until the limit event stops
                // the planner; tracking it would only push into the stop.
                resync(setpoint, measured);
                _dc->set(0);
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

        // Error alone also grows while accelerating, and velocity drops between slow steps;
        // only both, held for a while, mean the axis is pushing against something.
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
        _motor->softLimit(_stall_positive, true);
        return true;
    }

    void DCServo::release_stall() {
        _stalled = false;
        _motor->softLimit(_stall_positive, false);
    }

    // Releasing only outside position_deadband from the real target keeps the spring-back
    // from re-triggering the drive.
    bool DCServo::rest_overshoot(float error, float& loop_error) {
        if (_parked) {
            if (std::fabs(error) <= float(_position_deadband)) {
                return true;
            }
            _parked   = false;
            _rest_dir = error > 0.0f ? 1 : -1;
        }
        float overshoot = float(std::min(_overshoot_counts, _position_deadband));
        loop_error      = error + float(_rest_dir) * overshoot;
        if (_rest_dir == 0 || loop_error * float(_rest_dir) <= 0.0f) {
            _parked = true;
        }
        return _parked;
    }

    void DCServo::update() {
        EncoderSample sample   = _encoder->sample();
        int32_t       count    = _encoder_invert ? -sample.count : sample.count;
        float         measured = float(count);
        float         setpoint = float(get_axis_steps(_axis)) * _counts_per_step;

        update_filters();

        // Before the disabled path: a homing failure also disables the motor.
        if (_stalled && !_homing) {
            release_stall();
        }

        if (_tripped) {
            // Release only once the alarm has been raised, then cleared.  The trip raises a
            // hard stop (Critical), and a reset only passes through Alarm with unhomed axes.
            if (state_is(State::Alarm) || state_is(State::Critical)) {
                _trip_alarmed = true;
            } else if (_trip_alarmed) {
                _tripped = false;
            }
        }

        if (_disabled || _tripped) {
            _dc->coast();  // not set(0): that brakes with slow_decay
            resync(setpoint, measured);
            return;
        }

        // A jump faster than max_rate allows is a redefined position (homing, reset after an
        // alarm), not a move; chasing it would be a lurch.
        auto  axis     = Axes::_axis[_axis];
        float max_step = axis->_maxRate / 60.0f * axis->_stepsPerMm * _counts_per_step * _dt;
        if (std::fabs(setpoint - _last_setpoint) > 4.0f * max_step + 2.0f) {
            resync(setpoint, measured);
            return;
        }

        float setpoint_delta = setpoint - _last_setpoint;
        _setpoint_vel += (setpoint_delta / _dt - _setpoint_vel) * _sp_alpha;
        track_motion(setpoint_delta);
        float measured_rate = (measured - _last_measured) / _dt;
        if (sample.timed) {
            measured_rate = _edge_vel.update(count, sample.edge_time, _dt, float(_encoder->timer_hz()));
        } else if (_encoder->has_timing()) {
            // Edges kept the snapshot from settling, and the estimator
            // cannot skip a count without misreading the next span.
            _edge_vel.reset();
        }
        _measured_vel += (measured_rate - _measured_vel) * _vel_alpha;
        _last_setpoint = setpoint;
        _last_measured = measured;

        float error       = setpoint - (measured + _offset);
        bool  approaching = _homing && Machine::Homing::approach();
        // Approaching a hard stop is meant to build up error.
        if (!(approaching && _homing_stall_error) && _max_following_error && std::fabs(error) > float(_max_following_error)) {
            log_error(name() << " " << axisName() << " following error " << error << " counts exceeds " << _max_following_error);
            trip();
            return;
        }

        if (stall_check(setpoint, measured, error, approaching)) {
            return;
        }

        // Stiction leaves the last count or two uncorrected, so at rest the integral would only
        // wind up into a stalled-motor drive.  Idle rather than zero setpoint velocity, because
        // slow moves sit still for many ticks between steps.
        float velocity_cmd = 0.0f;
        float drive        = 0.0f;
        float drive_limit  = _homing ? _homing_max_drive : Pid::max_output;
        bool  at_rest      = _position_deadband && state_is(State::Idle);
        // At rest the motion direction lingers for a while; rest_overshoot() owns _rest_dir by then.
        if (_motion_dir != 0 && !at_rest) {
            _rest_dir = _motion_dir;
        }
        if (!at_rest) {
            _parked = false;
        }
        float loop_error = error;
        bool  coast;
        if (at_rest && _overshoot_counts) {
            coast = rest_overshoot(error, loop_error);
        } else {
            coast = at_rest && std::fabs(error) <= float(_position_deadband);
        }
        if (coast) {
            _pid->reset();
        } else {
            velocity_cmd = _setpoint_vel + _position_kp * loop_error;
            drive        = _pid->update(velocity_cmd, _measured_vel, float(_motion_dir), _dt, drive_limit);
        }

        // Jams under the following error threshold; relative to the active limit so it also
        // fires under homing_max_drive.  Stall homing approaches are left to stall_check.
        if (_stall_ms && !(approaching && _homing_stall_error) && std::fabs(drive) >= drive_limit * _stall_drive / 100.0f &&
            std::fabs(_measured_vel) < _stall_velocity) {
            ++_jam_ticks;
            if (float(_jam_ticks) * _dt * 1000.0f >= float(_stall_ms)) {
                log_error(name() << " " << axisName() << " stalled at " << drive << "% drive for " << _stall_ms << "ms, error " << error
                                 << " counts");
                trip();
                return;
            }
        } else {
            _jam_ticks = 0;
        }
        _dc->set(int32_t(drive * float(_dc->maxDuty()) / 100.0f));

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
        // Low-pass cutoff on the measured velocity.  With edge timing (see the startup
        // log) it only has to remove noise, and the A/B edge spacing ripple left while
        // the spacing is still being learnt after startup, so it can be high; without,
        // the velocity is quantized to whole counts per control period and needs a cutoff
        // well below the update rate.
        handler.item("velocity_filter_hz", _velocity_filter_hz, 1.0f, 10000.0f);

        // @config setpoint_filter_hz
        // @default 50
        // @tuning typical
        // Low-pass cutoff on the setpoint velocity that feeds the velocity loop forward.
        // It is quantized to whole steps per control period, counts_per_step/update_ms
        // apart, so a counts_per_step below 1 allows a higher cutoff and less lag.
        handler.item("setpoint_filter_hz", _setpoint_filter_hz, 1.0f, 10000.0f);

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
        // counts, the drive drops to zero (a brake with slow_decay) and the velocity PID
        // is reset.  Set it just above the
        // error stiction leaves at rest, so the loop does not hold drive on a stalled motor.
        // 0 disables it.
        handler.item("position_deadband", _position_deadband, 0, 10000);

        // @config overshoot_counts
        // @default 0
        // @tuning per-machine
        // With position_deadband set, the motor is driven this many encoder counts past the
        // target, in the direction it approached from, before it coasts, so the drive train
        // springing back leaves it on target: the spring-back seen with 0.  It then coasts
        // until the error leaves position_deadband, and is capped to it.  0 disables it.
        handler.item("overshoot_counts", _overshoot_counts, 0, 10000);

        // @config stall_ms
        // @default 100
        // @tuning per-machine
        // If the drive stays at or above stall_drive while the axis moves slower than
        // stall_velocity for this long, the motor coasts and the machine alarms, catching a
        // jam the following error check misses.  Must be longer than the axis takes to
        // accelerate past stall_velocity at full drive.  0 disables it.
        handler.item("stall_ms", _stall_ms, 0, 10000);

        // @config stall_drive
        // @default 90
        // @tuning per-machine
        // Drive, in percent of the current drive limit (homing_max_drive while homing),
        // at or above which the motor counts as pushing for stall detection.
        handler.item("stall_drive", _stall_drive, 1.0f, 100.0f);

        // @config stall_velocity
        // @default 100
        // @tuning per-machine
        // Measured speed, in encoder counts per second, below which the axis counts as
        // stopped for stall detection.
        handler.item("stall_velocity", _stall_velocity, 0.0f, 1e7f);

        // @config homing_stall_error
        // @default 0
        // @tuning per-machine
        // Homes against a hard stop instead of a switch: during a homing approach, a following
        // error above this many encoder counts, with the axis below homing_stall_velocity for
        // homing_stall_ms, raises this motor's limit.  Must exceed the error a normal approach
        // builds while accelerating.  0 disables it; the axis then needs limit switches.
        handler.item("homing_stall_error", _homing_stall_error, 0, 1000000);

        // @config homing_stall_velocity
        // @default 100
        // @tuning per-machine
        // Measured speed, in encoder counts per second, below which a homing approach counts
        // as stopped against the hard stop.
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
                        << "ms" << (_encoder_invert ? " inverted" : "") << (_encoder->has_timing() ? " edge timed" : " count differenced"));
    }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<DCServo> registration("dc_servo");
    }
}
