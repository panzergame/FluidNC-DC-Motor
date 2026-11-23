// Copyright 2022 - Mitch Bradley, Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
  PWM capabilities provided by the ESP32 LEDC controller via the ESP-IDF driver
*/
#include "Config.h"
#include "Driver/PwmPin.h"

#include <soc/ledc_struct.h>  // LEDC

#include <soc/soc_caps.h>
#include <driver/ledc.h>
#include <cstdint>

#if CONFIG_IDF_TARGET_ESP32  // ESP32/PICO-D4
#    include <esp32/rom/gpio.h>
#elif CONFIG_IDF_TARGET_ESP32S2
#    include <esp32s2/rom/gpio.h>
#elif CONFIG_IDF_TARGET_ESP32S3
#    include <esp32s3/rom/gpio.h>
#elif CONFIG_IDF_TARGET_ESP32C3
#    include <esp32c3/rom/gpio.h>
#else
#    error Target CONFIG_IDF_TARGET is not supported
#endif

//Use XTAL clock if possible to avoid timer frequency error when setting APB clock < 80 Mhz
//Need to be fixed in ESP-IDF
#ifdef SOC_LEDC_SUPPORT_XTAL_CLOCK
#    define LEDC_DEFAULT_CLK LEDC_USE_XTAL_CLK
#    define CLOCK_FREQUENCY 40000000
#else
#    define LEDC_DEFAULT_CLK LEDC_USE_APB_CLK
#    define CLOCK_FREQUENCY 80000000
#endif

// LEDC_CHANNEL_MAX and LEDC_TIMER_MAX count per speed mode; only the ESP32
// has a second (high speed) mode, which doubles both.
struct LedcTimer {
    uint32_t frequency;
    uint8_t  bits;
};

static LedcTimer ledcTimers[LEDC_SPEED_MODE_MAX][LEDC_TIMER_MAX]   = {};
static int       ledcTimersUsed[LEDC_SPEED_MODE_MAX]               = {};
static int       ledcChannelsUsed[LEDC_SPEED_MODE_MAX]             = {};

struct LedcAllocation {
    ledc_mode_t  mode;
    ledc_timer_t timer;
    int          channel;
    bool         newTimer;
};

// Pins at the same frequency share a timer, so the channel count rather than
// the timer count bounds how many PWM pins can exist.
static LedcAllocation allocateChannel(uint32_t frequency) {
    for (int mode = 0; mode < LEDC_SPEED_MODE_MAX; ++mode) {
        if (ledcChannelsUsed[mode] == LEDC_CHANNEL_MAX) {
            continue;
        }
        for (int timer = 0; timer < ledcTimersUsed[mode]; ++timer) {
            if (ledcTimers[mode][timer].frequency == frequency) {
                return { ledc_mode_t(mode), ledc_timer_t(timer), ledcChannelsUsed[mode]++, false };
            }
        }
    }
    for (int mode = 0; mode < LEDC_SPEED_MODE_MAX; ++mode) {
        if (ledcChannelsUsed[mode] < LEDC_CHANNEL_MAX && ledcTimersUsed[mode] < LEDC_TIMER_MAX) {
            int timer                      = ledcTimersUsed[mode]++;
            ledcTimers[mode][timer].frequency = frequency;
            return { ledc_mode_t(mode), ledc_timer_t(timer), ledcChannelsUsed[mode]++, true };
        }
    }
    Assert(false, "Out of LEDC PwmPin channels");
    return {};
}

// Calculate the highest PwmPin precision in bits for the desired frequency
// 80,000,000 (APB Clock) = freq * maxCount
// maxCount is a power of two between 2^1 and 2^20
// frequency is at most 80,000,000 / 2^1 = 40,000,000, limited elsewhere
// to 20,000,000 to give a period of at least 2^2 = 4 levels of control.
static uint8_t calc_pwm_precision(uint32_t frequency) {
    if (frequency == 0) {
        frequency = 1;  // Limited elsewhere but just to be safe...
    }

    // Increase the precision (bits) until it exceeds the frequency:
    const uint8_t  ledcMaxBits = LEDC_TIMER_BIT_MAX - 1;
    const uint32_t maxCount    = uint32_t(CLOCK_FREQUENCY) / frequency;
    for (uint32_t bits = 2; bits <= ledcMaxBits; ++bits) {
        if ((1u << bits) > maxCount) {
            return uint8_t(bits - 1);
        }
    }

    return ledcMaxBits;
}

PwmPin::PwmPin(pinnum_t gpio, bool isActiveLow, uint32_t frequency) : _gpio(gpio), _frequency(frequency) {
    auto        alloc     = allocateChannel(frequency);
    ledc_mode_t speedmode = alloc.mode;
    _channel              = objnum_t(int(speedmode) * LEDC_CHANNEL_MAX + alloc.channel);
    auto&       timer     = ledcTimers[speedmode][alloc.timer];

    // Reconfiguring a shared timer would restart the channels already on it.
    if (alloc.newTimer) {
        uint8_t bits = calc_pwm_precision(frequency);

        ledc_timer_config_t ledc_timer = {};
        ledc_timer.speed_mode          = speedmode;
        ledc_timer.duty_resolution     = ledc_timer_bit_t(bits);
        ledc_timer.timer_num           = alloc.timer;
        ledc_timer.freq_hz             = frequency;
        ledc_timer.clk_cfg             = LEDC_DEFAULT_CLK;

        size_t attempt = 0;
        for (attempt = 0; attempt < 5; ++attempt) {
            if (ledc_timer_config(&ledc_timer) != ESP_OK) {
                log_error("ledc timer setup failed. Frequency: " << frequency << " hz; duty resolution: " << bits);
                ledc_timer.duty_resolution = ledc_timer_bit_t(--bits);
            } else {
                break;
            }
        }
        if (attempt == 5) {
            Assert(false, "LEDC timer setup failed");
        }
        timer.bits = bits;
    }
    uint8_t bits = timer.bits;
    _period      = (1 << bits) - 1;

    uint32_t maxFrequency = uint32_t(CLOCK_FREQUENCY / float(1 << bits));
    log_info("    Max frequency of LEDC set at " << maxFrequency << "; duty resolution: " << bits << "; channel " << _channel << "; timer "
                                                 << alloc.timer);

    ledc_channel_config_t ledc_channel = { .gpio_num   = int(_gpio),
                                           .speed_mode = speedmode,
                                           .channel    = ledc_channel_t(alloc.channel),
                                           .intr_type  = LEDC_INTR_DISABLE,
                                           .timer_sel  = alloc.timer,
                                           .duty       = 0,
                                           .hpoint     = 0,
                                           .flags      = { .output_invert = isActiveLow } };

    if (ledc_channel_config(&ledc_channel) != ESP_OK) {
        log_error("ledc channel setup failed. Frequency: " << frequency << " hz; duty resolution: " << bits << "; channel: " << _channel);
    }

    // We write 1 item to ensure the complete configuration is correct of both timer and ledc:
    ledc_set_duty(speedmode, ledc_channel_t(alloc.channel), 0);
    ledc_update_duty(speedmode, ledc_channel_t(alloc.channel));
}

// cppcheck-suppress unusedFunction
void IRAM_ATTR PwmPin::setDuty(uint32_t duty) {
    // This is like ledcWrite, but it is called from an ISR
    // and ledcWrite uses RTOS features not compatible with ISRs
    // Also, ledcWrite infers enable from duty, which is incorrect
    // for use with RcServo which wants the

    // We might be able to use ledc_duty_config() which is IRAM_ATTR

    objnum_t    c         = _channel % LEDC_CHANNEL_MAX;
    ledc_mode_t speedmode = ledc_mode_t(_channel / LEDC_CHANNEL_MAX);
    bool        on        = duty != 0;

    // ledc_set_duty(speedmode, ledc_channel_t(c), duty);
    // This does:

    auto& ch = LEDC.channel_group[speedmode].channel[c];

    // These can be ignored because they should have been set by initial set_duty / update_duty (ctor):
    // ch.conf1.duty_inc   = 1;
    // ch.conf1.duty_num   = 1;
    // ch.conf1.duty_cycle = 1;
    // ch.conf1.duty_scale = 0;
    ch.duty.duty = duty << 4;

    // -> ledc_hal_set_sig_out_en(&(p_ledc_obj[speed_mode]->ledc_hal), channel, true);
    ch.conf0.sig_out_en = on;
    // -> ledc_hal_set_duty_start(&(p_ledc_obj[speed_mode]->ledc_hal), channel, true);
    while (ch.conf1.duty_start) {}
    ch.conf1.duty_start = on;
    // -> ledc_ls_channel_update(speed_mode, channel); // Doesn't seem to hurt for high speed channels.
    ch.conf0.low_speed_update = 1;

    // log_debug("Setting duty to " << duty);
}

PwmPin::~PwmPin() {
    // XXX we need to release the channel so it can be reused by later calls to pwmInit
#define MATRIX_DETACH_OUT_SIG 0x100
    gpio_matrix_out(_gpio, MATRIX_DETACH_OUT_SIG, false, false);
}
