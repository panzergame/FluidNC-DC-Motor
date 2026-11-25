// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "EdgeVelocity.h"

#include <algorithm>  // std::clamp, std::min
#include <utility>    // std::swap
#include <cmath>      // std::fabs, std::copysign
#include <cstdlib>    // std::abs

namespace MotorDrivers {
    // Longest a span may stretch to.  It also keeps every edge time
    // difference far from the capture timer wrap.
    static constexpr float max_window_s = 0.02f;

    // With no edge for this long the axis is at rest.
    static constexpr float idle_timeout_s = 1.0f;

    // Per-cycle weight of the edge spacing estimate: slow enough to average
    // out real speed ripple, fast enough to settle within the first moves.
    static constexpr float spacing_alpha = 0.02f;

    // A single interval this far off a quarter of its cycle is speed change,
    // not edge spacing.
    static constexpr float min_spacing = 0.25f;
    static constexpr float max_spacing = 2.5f;

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

    float& EdgeVelocity::spacing(int32_t edge) {
        return _spacing[_direction > 0 ? 0 : 1][((edge % 4) + 4) % 4];
    }

    float EdgeVelocity::distance(int32_t from, int32_t to) {
        int32_t lo = edge_of(from);
        int32_t hi = edge_of(to);
        if (lo > hi) {
            std::swap(lo, hi);
        }
        float d = 0.0f;
        for (int32_t e = lo; e < hi; ++e) {
            d += spacing(e);
        }
        return d;
    }

    // Needs a full quadrature cycle of one-count steps: over its four
    // intervals the uneven spacing cancels, so each interval's share of the
    // cycle time is its share of the four counts.  Marks are taken once per
    // control period, so this only learns at speeds below one count per
    // period - which is where a one-interval span needs it.
    void EdgeVelocity::learn() {
        if (_n_marks < history) {
            return;
        }
        for (int i = 0; i < history - 1; ++i) {
            if (std::abs(back(i).count - back(i + 1).count) != 1) {
                return;
            }
        }
        uint32_t cycle = back(0).time - back(history - 1).time;
        if (cycle == 0) {
            return;
        }
        float share = 4.0f * float(back(0).time - back(1).time) / float(cycle);
        if (share < min_spacing || share > max_spacing) {
            return;
        }
        int32_t edge = std::min(edge_of(back(0).count), edge_of(back(1).count));
        float&  s    = spacing(edge);
        s += (share - s) * spacing_alpha;

        float* row = _spacing[_direction > 0 ? 0 : 1];
        float  sum = row[0] + row[1] + row[2] + row[3];
        for (int i = 0; i < 4; ++i) {
            row[i] = std::clamp(row[i] * 4.0f / sum, min_spacing, max_spacing);
        }
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
        learn();

        // The span reaches back just over one control period: a span of fixed counts grows to
        // many periods at low speed, and that delay makes the velocity loop oscillate.  So short
        // a span exposes the uneven edge spacing, hence the learnt weights in distance().
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

        uint32_t ticks = ref ? edge_time - ref->time : 0;
        if (ticks != 0) {
            _velocity = float(direction) * distance(ref->count, count) * timer_hz / float(ticks);
        }
        return _velocity;
    }
}
