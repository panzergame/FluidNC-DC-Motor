// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>

namespace MotorDrivers {
    /*
        Encoder velocity from edge timestamps (synchronous M/T method): the counts between
        two edges over the time between those same edges, so low speeds are not quantized
        to whole counts per control period.
    */
    class EdgeVelocity {
    public:
        void reset() { _primed = false; }
        // After a count that could not be timed: the marks no longer form one run of
        // edges, but the speed and direction hold until the next span.
        void skip() { _n_marks = 0; }

        // count and edge_time must come from one consistent encoder sample;
        // edge_time is in ticks of timer_hz.  Returns counts per second.
        float update(int32_t count, uint32_t edge_time, float dt, float timer_hz);

    private:
        struct Mark {
            int32_t  count;
            uint32_t time;
        };

        // Marks
        void push(int32_t count, uint32_t edge_time);
        void restart(int32_t count, uint32_t edge_time);
        // i edges' worth of marks back from the newest.
        const Mark& back(int i) const { return _marks[(_head + history - i) % history]; }

        // Velocity
        float       idle(float dt);
        const Mark* span_start(uint32_t edge_time, float dt, float timer_hz) const;

        // Enough marks for a span to reach one control period back when the
        // newest edges fell close together.
        static constexpr int history = 5;

        Mark    _marks[history];
        int     _head      = 0;
        int     _n_marks   = 0;
        int32_t _last      = 0;
        int     _direction = 0;
        float   _idle_s    = 0.0f;
        float   _velocity  = 0.0f;
        bool    _primed    = false;
    };
}
