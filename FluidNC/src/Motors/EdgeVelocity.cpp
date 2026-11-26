// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "EdgeVelocity.h"

#include <cmath>  // std::fabs, std::copysign

namespace MotorDrivers {
    // Longest a span may stretch to.  It also keeps every edge time
    // difference far from the capture timer wrap.
    static constexpr float max_window_s = 0.02f;

    // With no edge for this long the axis is at rest.
    static constexpr float idle_timeout_s = 1.0f;

    void EdgeVelocity::push(int32_t count, uint32_t edge_time) {
        _head         = (_head + 1) % history;
        _marks[_head] = { count, edge_time };
        if (_n_marks < history) {
            ++_n_marks;
        }
    }

    void EdgeVelocity::restart(int32_t count, uint32_t edge_time) {
        _n_marks = 0;
        push(count, edge_time);
        _velocity = 0.0f;
    }

    float EdgeVelocity::idle(float dt) {
        _idle_s += dt;
        if (_idle_s >= idle_timeout_s) {
            _n_marks   = 0;
            _direction = 0;
            _velocity  = 0.0f;
            return 0.0f;
        }
        // Holding the last value would hide a stop.  The next edge has not
        // come yet, so the speed is at most one count over the time since
        // the last one - which may have been at the very end of its
        // period, hence one period less.
        float since = _idle_s - dt;
        if (since > 0.0f && std::fabs(_velocity) > 1.0f / since) {
            _velocity = std::copysign(1.0f / since, _velocity);
        }
        return _velocity;
    }

    // The span reaches back just over one control period: a span of fixed counts grows to
    // many periods at low speed, and that delay makes the velocity loop oscillate.
    const EdgeVelocity::Mark* EdgeVelocity::span_start(uint32_t edge_time, float dt, float timer_hz) const {
        uint32_t    min_window = uint32_t(dt * timer_hz);
        uint32_t    max_window = uint32_t(max_window_s * timer_hz);
        const Mark* ref        = nullptr;
        for (int i = 1; i < _n_marks; ++i) {
            const Mark& m       = back(i);
            uint32_t    elapsed = edge_time - m.time;
            if (ref && elapsed > max_window) {
                break;
            }
            ref = &m;
            if (elapsed >= min_window) {
                break;
            }
        }
        return ref;
    }

    float EdgeVelocity::update(int32_t count, uint32_t edge_time, float dt, float timer_hz) {
        if (!_primed) {
            // The last edge may be arbitrarily old, so it cannot anchor a span.
            _primed    = true;
            _last      = count;
            _n_marks   = 0;
            _direction = 0;
            _idle_s    = 0.0f;
            _velocity  = 0.0f;
            return 0.0f;
        }

        int32_t delta = count - _last;
        _last         = count;

        if (delta == 0) {
            return idle(dt);
        }
        _idle_s = 0.0f;

        // A span across a reversal measures neither direction, and chatter
        // on a single edge reverses within microseconds, which would read as
        // a huge speed.  This also seeds the first span after rest.
        int direction = delta > 0 ? 1 : -1;
        if (direction != _direction) {
            _direction = direction;
            restart(count, edge_time);
            return 0.0f;
        }

        push(count, edge_time);

        const Mark* ref   = span_start(edge_time, dt, timer_hz);
        uint32_t    ticks = ref ? edge_time - ref->time : 0;
        if (ticks != 0) {
            _velocity = float(count - ref->count) * timer_hz / float(ticks);
        }
        return _velocity;
    }
}
