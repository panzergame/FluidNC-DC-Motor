// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "EdgeVelocity.h"

#include <cmath>    // std::fabs, std::copysign
#include <cstdlib>  // std::abs

namespace MotorDrivers {
    // A full quadrature cycle, over which the uneven spacing of the A and B
    // edges cancels out.
    static constexpr int32_t span_counts = 4;

    // Longest a span may stretch to reach span_counts.  At speeds where four
    // counts take longer, the lag of a wider window costs more than the edge
    // spacing ripple it removes.
    static constexpr float max_window_s = 0.02f;

    // With no edge for this long the axis is at rest.  It also keeps every
    // edge time difference far from the capture timer wrap.
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

        const int32_t delta = count - _last;
        _last               = count;

        if (delta == 0) {
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
            const float since = _idle_s - dt;
            if (since > 0.0f && std::fabs(_velocity) > 1.0f / since) {
                _velocity = std::copysign(1.0f / since, _velocity);
            }
            return _velocity;
        }
        _idle_s = 0.0f;

        // A span across a reversal measures neither direction, and chatter
        // on a single edge reverses within microseconds, which would read as
        // a huge speed.  This also seeds the first span after rest.
        const int direction = delta > 0 ? 1 : -1;
        if (direction != _direction) {
            _direction = direction;
            restart(count, edge_time);
            return 0.0f;
        }

        push(count, edge_time);

        const uint32_t max_window = uint32_t(max_window_s * timer_hz);
        const Mark*    ref        = nullptr;
        for (int i = 1; i < _n_marks; ++i) {
            const Mark& m = back(i);
            if (ref && edge_time - m.time > max_window) {
                break;
            }
            ref = &m;
            if (std::abs(count - m.count) >= span_counts) {
                break;
            }
        }

        const uint32_t ticks = ref ? edge_time - ref->time : 0;
        if (ticks != 0) {
            _velocity = float(count - ref->count) * timer_hz / float(ticks);
        }
        return _velocity;
    }
}
