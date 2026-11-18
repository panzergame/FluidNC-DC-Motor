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

        The span only reaches back far enough to cover one control period.
        A span of a fixed number of counts would grow to many periods at low
        speed, and the delay that adds is enough to make the velocity loop
        oscillate there.  A span that short exposes the uneven spacing of the
        quadrature edges - A and B never have exactly 50% duty and 90 degrees
        of phase - so each of the four spacings is learnt, per direction, and
        the counts in a span are weighted by them.
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

        // Distance from each edge to the next, in counts, indexed by
        // direction and by edge index mod 4; each row sums to 4.
        float _spacing[2][4] = { { 1.0f, 1.0f, 1.0f, 1.0f }, { 1.0f, 1.0f, 1.0f, 1.0f } };
    };
}
