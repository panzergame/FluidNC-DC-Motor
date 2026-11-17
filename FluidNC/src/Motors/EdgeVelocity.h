// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <stdint.h>

namespace MotorDrivers {
    /*
        Encoder velocity from edge timestamps (synchronous M/T method).

        Differencing the count once per control period quantizes velocity to
        whole counts per period, which at low speed alternates between zero
        and one-count spikes.  Dividing the counts between two edges by the
        time between those same edges is exact instead: it reduces to the
        edge period at low speed and to the mean speed over the period at
        high speed, without switching between the two.
    */
    class EdgeVelocity {
    public:
        void reset() { _primed = false; }

        // count and edge_time must come from one consistent encoder sample;
        // edge_time is in ticks of timer_hz.  Returns counts per second.
        float update(int32_t count, uint32_t edge_time, float dt, float timer_hz);

    private:
        struct Mark {
            int32_t  count;
            uint32_t time;
        };

        void restart(int32_t count, uint32_t edge_time);
        void push(int32_t count, uint32_t edge_time);
        // i edges' worth of marks back from the newest.
        const Mark& back(int i) const { return _marks[(_head + history - i) % history]; }

        // Up to span_counts back from the newest mark, plus the newest.
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
