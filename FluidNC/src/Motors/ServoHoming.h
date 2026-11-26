// Copyright (c) 2026 -	Tristan Porteries
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"

namespace MotorDrivers {
    // Homing settings of a closed-loop motor: the drive cap and, optionally, stall homing
    // against a hard stop.
    class ServoHoming : public Configuration::Configurable {
    public:
        uint32_t _stall_error    = 0;
        float    _stall_velocity = 100.0f;
        uint32_t _stall_ms       = 50;
        uint32_t _release_counts = 10;
        float    _max_drive      = 100.0f;

        // Configuration handlers:
        void group(Configuration::HandlerBase& handler) override;
    };
}
