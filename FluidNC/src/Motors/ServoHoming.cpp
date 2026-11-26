// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "ServoHoming.h"

#include "Configuration/HandlerBase.h"

namespace MotorDrivers {
    void ServoHoming::group(Configuration::HandlerBase& handler) {
        // @config stall_error
        // @default 0
        // @tuning per-machine
        // Homes against a hard stop instead of a switch: during a homing approach, a following
        // error above this many encoder counts, with the axis below stall_velocity for
        // stall_ms, raises this motor's limit.  Must exceed the error a normal approach
        // builds while accelerating.  0 disables it; the axis then needs limit switches.
        handler.item("stall_error", _stall_error, 0, 1000000);

        // @config stall_velocity
        // @default 100
        // @tuning per-machine
        // Measured speed, in encoder counts per second, below which a homing approach counts
        // as stopped against the hard stop.
        handler.item("stall_velocity", _stall_velocity, 0.0f, 1e7f);

        // @config stall_ms
        // @default 50
        // @tuning per-machine
        // How long the stall condition has to hold before the limit is raised.
        handler.item("stall_ms", _stall_ms, 1, 10000);

        // @config release_counts
        // @default 10
        // @tuning per-machine
        // After a stall, the soft limit clears once the axis has backed this many encoder
        // counts off the stop.  Must be less than the motor's pulloff_mm in counts, or
        // homing fails with the limit still engaged.
        handler.item("release_counts", _release_counts, 1, 1000000);

        // @config max_drive
        // @default 100
        // @tuning per-machine
        // Caps the drive, in percent, for the whole homing cycle, which bounds the force
        // the axis pushes into a hard stop with.  Leave enough to overcome friction at
        // the homing feed rate.
        handler.item("max_drive", _max_drive, 1.0f, 100.0f);
    }
}
