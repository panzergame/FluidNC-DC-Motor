// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    A brushed DC motor with a quadrature encoder, run as a servo.

    The planner and step segment generator already enforce acceleration and max_rate,
    and the axis step count follows that profile whether or not a stepper is attached.
    That count is the position setpoint, so this driver does no profiling, only tracking:

        velocity command = setpoint velocity + position_kp * position error
        drive            = velocity_pid(velocity command, measured velocity)

    There is no index pulse, so homing needs limit switches, or homing/stall_error to
    home against a hard stop: when the axis stops following the setpoint during a homing
    approach, the motor's limit is raised in software as if a switch had fired.

    Example YAML

      dc_servo:
        position_kp: 20
        max_following_error: 200
        encoder:
          a_pin: gpio.34
          b_pin: gpio.35
          counts_per_step: 1
        homing:
          stall_error: 100
          stall_velocity: 50
          stall_ms: 50
          max_drive: 30
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

        if (!_encoder->init()) {
            log_config_error("    " << name() << " disabled: no encoder");
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
        _homing_mode = isHoming;
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

    // A jump faster than max_rate allows is a redefined position (homing, reset after an
    // alarm), not a move; chasing it would be a lurch.
    bool DCServo::setpoint_jumped(float setpoint_delta) const {
        auto  axis     = Axes::_axis[_axis];
        float max_step = axis->_maxRate / 60.0f * axis->_stepsPerMm * _encoder->counts_per_step() * _dt;
        return std::fabs(setpoint_delta) > 4.0f * max_step + 2.0f;
    }

    void DCServo::update_velocities(float setpoint, float setpoint_delta, float measured, const EncoderSample& sample) {
        _setpoint_vel += (setpoint_delta / _dt - _setpoint_vel) * _sp_alpha;
        track_motion(setpoint_delta);

        float measured_rate = (measured - _last_measured) / _dt;
        if (sample.timed) {
            measured_rate = _edge_vel.update(sample.count, sample.edge_time, _dt, float(_encoder->timer_hz()));
        } else if (_encoder->has_timing()) {
            // Edges kept the snapshot from settling; a span across the untimed
            // count would be misread, so only the speed carries over.
            _edge_vel.skip();
        }
        _measured_vel += (measured_rate - _measured_vel) * _vel_alpha;

        _last_setpoint = setpoint;
        _last_measured = measured;
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

    // Release only once the alarm has been raised, then cleared.  The trip raises a
    // hard stop (Critical), and a reset only passes through Alarm with unhomed axes.
    void DCServo::update_trip() {
        if (!_tripped) {
            return;
        }
        if (state_is(State::Alarm) || state_is(State::Critical)) {
            _trip_alarmed = true;
        } else if (_trip_alarmed) {
            _tripped = false;
        }
    }

    bool DCServo::following_error_exceeded(float error) {
        if (_max_following_error && std::fabs(error) > float(_max_following_error)) {
            log_error(name() << " " << axisName() << " following error " << error << " counts exceeds " << _max_following_error);
            return true;
        }
        return false;
    }

    // Jams under the following error threshold; relative to the active limit so it also
    // fires under homing/max_drive.  Stall homing approaches are left to stall_check.
    bool DCServo::jammed(float drive, float drive_limit, float error, bool stall_homing) {
        if (_stall_ms && !stall_homing && std::fabs(drive) >= drive_limit * _stall_drive / 100.0f &&
            std::fabs(_measured_vel) < _stall_velocity) {
            ++_jam_ticks;
        } else {
            _jam_ticks = 0;
            return false;
        }
        if (elapsed_ms(_jam_ticks) < float(_stall_ms)) {
            return false;
        }
        log_error(name() << " " << axisName() << " stalled at " << drive << "% drive for " << _stall_ms << "ms, error " << error
                         << " counts");
        return true;
    }

    bool DCServo::stall_check(float setpoint, float measured, float error, bool approaching) {
        if (_stalled) {
            return hold_stall(setpoint, measured, approaching);
        }
        if (!detect_stall(error, approaching)) {
            return false;
        }
        _stall_ticks    = 0;
        _stalled        = true;
        _stall_positive = error > 0.0f;
        _stall_pos      = measured;
        log_debug(name() << " " << axisName() << " stalled at " << (measured + _offset) << " error " << error);
        resync(setpoint, measured);
        _dc->brake();
        _motor->softLimit(_stall_positive, true);
        return true;
    }

    bool DCServo::hold_stall(float setpoint, float measured, bool approaching) {
        if (approaching) {
            // The setpoint keeps advancing until the limit event stops
            // the planner; tracking it would only push into the stop.
            resync(setpoint, measured);
            _dc->brake();
            return true;
        }
        if (std::fabs(measured - _stall_pos) > float(_homing->_release_counts)) {
            release_stall();
        }
        return false;
    }

    // Error alone also grows while accelerating, and velocity drops between slow steps;
    // only both, held for a while, mean the axis is pushing against something.
    bool DCServo::detect_stall(float error, bool approaching) {
        if (!approaching || !can_self_home()) {
            _stall_ticks = 0;
            return false;
        }
        if (std::fabs(error) > float(_homing->_stall_error) && std::fabs(_measured_vel) < _homing->_stall_velocity) {
            ++_stall_ticks;
        } else {
            _stall_ticks = 0;
            return false;
        }
        return elapsed_ms(_stall_ticks) >= float(_homing->_stall_ms);
    }

    void DCServo::release_stall() {
        _stalled = false;
        _motor->softLimit(_stall_positive, false);
    }

    // Stiction leaves the last count or two uncorrected, so at rest the integral would only
    // wind up into a stalled-motor drive.  Idle rather than zero setpoint velocity, because
    // slow moves sit still for many ticks between steps.
    bool DCServo::rest_brake(float error, float& loop_error) {
        if (!_position_deadband || !state_is(State::Idle)) {
            _parked = false;
            // At rest the motion direction lingers for a while; rest_overshoot() owns _rest_dir by then.
            if (_motion_dir != 0) {
                _rest_dir = _motion_dir;
            }
            return false;
        }
        if (_overshoot_counts) {
            return rest_overshoot(error, loop_error);
        }
        return std::fabs(error) <= float(_position_deadband);
    }

    // Parked, the band runs from position_deadband short of the target to position_deadband
    // past the overshoot point, so neither the overrun nor the spring-back re-drives.
    bool DCServo::rest_overshoot(float error, float& loop_error) {
        if (_parked) {
            float e = _rest_dir ? error * float(_rest_dir) : std::fabs(error);  // positive short of the target
            if (e <= float(_position_deadband) && e >= -float(_overshoot_counts + _position_deadband)) {
                return true;
            }
            _parked   = false;
            _rest_dir = error > 0.0f ? 1 : -1;
        }
        loop_error = error + float(_rest_dir) * float(_overshoot_counts);
        if (_rest_dir == 0 || loop_error * float(_rest_dir) <= 0.0f) {
            _parked = true;
        }
        return _parked;
    }

    void DCServo::report(float setpoint, float measured, float error, float velocity_cmd, float drive) {
        if (!_report_ms) {
            return;
        }
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

    void DCServo::update() {
        EncoderSample sample   = _encoder->sample();
        float         measured = float(sample.count);
        float         setpoint = float(get_axis_steps(_axis)) * _encoder->counts_per_step();

        update_filters();

        // Before the disabled path: a homing failure also disables the motor.
        if (_stalled && !_homing_mode) {
            release_stall();
        }
        update_trip();


        if (_disabled || _tripped) {
            _dc->coast();  // not set(0): that brakes
            resync(setpoint, measured);
            return;
        }

        float setpoint_delta = setpoint - _last_setpoint;
        if (setpoint_jumped(setpoint_delta)) {
            resync(setpoint, measured);
            return;
        }

        update_velocities(setpoint, setpoint_delta, measured, sample);

        float error        = setpoint - (measured + _offset);
        bool  approaching  = _homing_mode && Machine::Homing::approach();
        bool  stall_homing = approaching && can_self_home();  // left to stall_check()
        // Approaching a hard stop is meant to build up error.
        if (!stall_homing && following_error_exceeded(error)) {
            trip();
            return;
        }

        if (stall_check(setpoint, measured, error, approaching)) {
            return;
        }

        float drive_limit  = _homing_mode && _homing ? _homing->_max_drive : Pid::max_output;
        float loop_error   = error;
        float velocity_cmd = 0.0f;
        float drive        = 0.0f;
        if (rest_brake(error, loop_error)) {
            _pid->reset();
        } else {
            velocity_cmd = _setpoint_vel + _position_kp * loop_error;
            drive        = _pid->update(velocity_cmd, _measured_vel, float(_motion_dir), _dt, drive_limit);
        }

        if (jammed(drive, drive_limit, error, stall_homing)) {
            trip();
            return;
        }
        _dc->set(int32_t(drive * float(_dc->maxDuty()) / 100.0f));

        report(setpoint, measured, error, velocity_cmd, drive);
    }

    void DCServo::validate() {
        Assert(_dc != nullptr, "dc_motor section is required");
        Assert(_pid != nullptr, "velocity_pid section is required");
        Assert(_encoder != nullptr, "encoder section is required");
        if (_overshoot_counts && !_position_deadband) {
            log_warn(name() << " overshoot_counts has no effect without position_deadband");
        }
    }

    void DCServo::group(Configuration::HandlerBase& handler) {
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
        // log) it only has to remove noise and the ripple of the uneven A/B edge spacing,
        // so it can be high; without,
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
        // @default 100
        // @tuning per-machine
        // Position error, in encoder counts, beyond which the motor coasts and the machine
        // alarms.  0 disables the check.
        handler.item("max_following_error", _max_following_error, 0, 1000000);

        // @config position_deadband
        // @default 0
        // @tuning per-machine
        // While the machine is idle and the position error is within this many encoder
        // counts, the drive drops to zero, braking the motor, and the velocity PID is
        // reset.  Set it just above the error stiction leaves at rest, so the loop does
        // not hold drive on a stalled motor.
        // 0 disables it.
        handler.item("position_deadband", _position_deadband, 0, 10000);

        // @config overshoot_counts
        // @default 0
        // @tuning per-machine
        // With position_deadband set, the motor is driven this many encoder counts past the
        // target, in the direction it approached from, before the drive drops, so the drive
        // train springing back leaves it on target: the spring-back seen with 0.  It then
        // stays parked until the error is more than position_deadband short of the target or
        // past the overshoot point, so position_deadband need not clear the overshoot.
        // 0 disables it.
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
        // Drive, in percent of the current drive limit (homing/max_drive while homing),
        // at or above which the motor counts as pushing for stall detection.
        handler.item("stall_drive", _stall_drive, 1.0f, 100.0f);

        // @config stall_velocity
        // @default 100
        // @tuning per-machine
        // Measured speed, in encoder counts per second, below which the axis counts as
        // stopped for stall detection.
        handler.item("stall_velocity", _stall_velocity, 0.0f, 1e7f);

        // @config report_ms
        // @default 0
        // Interval for logging setpoint, position, peak following error and drive while
        // tuning.  0 disables it.
        handler.item("report_ms", _report_ms, 0, 60000);

        handler.section("encoder", _encoder);
        handler.section("homing", _homing);
        handler.section("dc_motor", _dc);
        handler.section("velocity_pid", _pid);

        Servo::group(handler);
    }

    void DCServo::config_message() {
        log_info("    " << name() << " Encoder A:" << _encoder->a_pin().name() << " B:" << _encoder->b_pin().name() << " period:" << _update_ms
                        << "ms" << (_encoder->inverted() ? " inverted" : "") << (_encoder->has_timing() ? " edge timed" : " count differenced"));
    }

    // Configuration registration
    namespace {
        MotorFactory::InstanceBuilder<DCServo> registration("dc_servo");
    }
}
