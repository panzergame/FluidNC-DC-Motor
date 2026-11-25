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
        // Forgets the motion, not the learnt edge spacing.
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

        // Edge index of the edge that produced a count, in the direction of
        // travel: counting up, count c was reached by the edge between c-1
        // and c; counting down, by the edge between c and c+1.
        int32_t edge_of(int32_t count) const { return _direction > 0 ? count : count + 1; }
        float&  spacing(int32_t edge);
        // Distance between the edges that produced two counts, in counts,
        // corrected for the uneven edge spacing.
        float distance(int32_t from, int32_t to);
        void  learn();

        // A full quadrature cycle of single-count marks, plus the newest.
        static constexpr int history = 5;

        Mark    _marks[history];
        int     _head      = 0;
        int     _n_marks   = 0;
        int32_t _last      = 0;
        int     _direction = 0;
        float   _idle_s    = 0.0f;
        float   _velocity  = 0.0f;
        bool    _primed    = false;

        // Distance from each edge to the next, in counts, indexed by direction and by edge
        // index mod 4; each row sums to 4.  A and B never have exactly 50% duty and 90 degrees.
        float _spacing[2][4] = { { 1.0f, 1.0f, 1.0f, 1.0f }, { 1.0f, 1.0f, 1.0f, 1.0f } };
    };
}
