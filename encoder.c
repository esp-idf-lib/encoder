/*
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2026 Chris Leishman (@cleishm)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file encoder.c
 *
 * ESP-IDF HW timer-based driver for rotary encoders
 *
 * Copyright (c) 2019 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2026 Chris Leishman (@cleishm)
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "encoder.h"
#include <stdlib.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

static const char *TAG = "encoder";
static const int8_t valid_states[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define PIN_VALID(p) ((p) >= 0 && (p) < GPIO_NUM_MAX)

// The polling interval is a build time constant, not a part of the
// configuration: the button timings are counted in polling intervals, so the
// value has to be the same for the whole run and cannot be changed per encoder
#define RE_POLLING_INTERVAL_US ((uint32_t)CONFIG_RE_INTERVAL_US)

typedef struct
{
    int64_t last_time;
    uint16_t coeff;
} rotary_encoder_acceleration_t;

struct rotary_encoder
{
    gpio_num_t pin_a;                    //!< Encoder pin A
    gpio_num_t pin_b;                    //!< Encoder pin B
    gpio_num_t pin_btn;                  //!< Button pin, or GPIO_NUM_NC if unused
    uint8_t btn_pressed_level;           //!< GPIO level when button is pressed (0 or 1)
    uint32_t btn_dead_time_us;           //!< Button dead time in microseconds
    uint32_t btn_long_press_time_us;     //!< Long press threshold in microseconds
    uint32_t acceleration_threshold_ms;  //!< Acceleration threshold in milliseconds (acceleration starts below this interval)
    uint32_t acceleration_cap_ms;        //!< Acceleration cap in milliseconds (minimum interval, limits max acceleration)
    rotary_encoder_event_cb_t callback;  //!< Event callback
    void *callback_ctx;                  //!< User context passed to callback
    esp_timer_handle_t timer;
    SemaphoreHandle_t lock;
    uint8_t code;
    uint16_t store;
    uint64_t btn_pressed_time_us;
    rotary_encoder_btn_state_t btn_state;
    rotary_encoder_acceleration_t acceleration;
    TaskHandle_t callback_task;          //!< Task the callback runs in, NULL when idle
};

inline static void read_encoder(rotary_encoder_handle_t handle)
{
    rotary_encoder_event_t ev =
    {
        .sender = handle
    };

    if (PIN_VALID(handle->pin_btn))
        do
        {
            if (handle->btn_state == RE_BTN_PRESSED && handle->btn_pressed_time_us < handle->btn_dead_time_us)
            {
                // Dead time
                handle->btn_pressed_time_us += RE_POLLING_INTERVAL_US;
                break;
            }

            // read button state
            if (gpio_get_level(handle->pin_btn) == handle->btn_pressed_level)
            {
                if (handle->btn_state == RE_BTN_RELEASED)
                {
                    // first press
                    handle->btn_state = RE_BTN_PRESSED;
                    handle->btn_pressed_time_us = 0;
                    ev.type = RE_ET_BTN_PRESSED;
                    handle->callback(&ev, handle->callback_ctx);
                    break;
                }

                handle->btn_pressed_time_us += RE_POLLING_INTERVAL_US;

                if (handle->btn_state == RE_BTN_PRESSED && handle->btn_pressed_time_us >= handle->btn_long_press_time_us)
                {
                    // Long press
                    handle->btn_state = RE_BTN_LONG_PRESSED;
                    ev.type = RE_ET_BTN_LONG_PRESSED;
                    handle->callback(&ev, handle->callback_ctx);
                }
            }
            else if (handle->btn_state != RE_BTN_RELEASED)
            {
                bool clicked = handle->btn_state == RE_BTN_PRESSED;
                // released
                handle->btn_state = RE_BTN_RELEASED;
                ev.type = RE_ET_BTN_RELEASED;
                handle->callback(&ev, handle->callback_ctx);
                if (clicked)
                {
                    ev.type = RE_ET_BTN_CLICKED;
                    handle->callback(&ev, handle->callback_ctx);
                }
            }
        }
        while (0);

    handle->code <<= 2;
    handle->code |= gpio_get_level(handle->pin_a);
    handle->code |= gpio_get_level(handle->pin_b) << 1;
    handle->code &= 0xf;

    if (!valid_states[handle->code])
        return;

    int8_t inc = 0;

    handle->store = (handle->store << 4) | handle->code;

    if ((handle->store == 0xe817) || (handle->store == 0x17e8)) inc = 1;
    if ((handle->store == 0xd42b) || (handle->store == 0x2bd4)) inc = -1;

    if (inc)
    {
        ev.diff = inc;
        if (handle->acceleration.coeff > 1)
        {
            int64_t nowMicros = esp_timer_get_time();
            uint32_t accelerationThresholdMs = handle->acceleration_threshold_ms;
            uint32_t accelerationCapMs = handle->acceleration_cap_ms;
            uint32_t millisAfterLastMotion = (nowMicros - handle->acceleration.last_time) / 1000u;
            handle->acceleration.last_time = nowMicros;

            if (millisAfterLastMotion < accelerationThresholdMs)
            {
                if (millisAfterLastMotion < accelerationCapMs)
                {
                    millisAfterLastMotion = accelerationCapMs; // limit to maximum acceleration
                }
                if (!millisAfterLastMotion)
                {
                    // Two detents landed in the same millisecond, which happens
                    // when the timer task catches up after a delay
                    millisAfterLastMotion = 1;
                }
                ev.diff = inc * ((int32_t)(handle->acceleration.coeff / millisAfterLastMotion) == 0 ? 1 : (int32_t)(handle->acceleration.coeff / millisAfterLastMotion));
            }
        }

        handle->store = 0;
        ev.type = RE_ET_CHANGED;
        handle->callback(&ev, handle->callback_ctx);
    }
}

static bool is_callback_task(rotary_encoder_handle_t handle)
{
    return handle->callback_task && handle->callback_task == xTaskGetCurrentTaskHandle();
}

static void encoder_timer_handler(void *arg)
{
    rotary_encoder_handle_t handle = (rotary_encoder_handle_t)arg;

    // Remember the task, so the library can tell its own callback from any
    // other caller and refuse the calls that would deadlock or free the handle
    // under the callback
    handle->callback_task = xTaskGetCurrentTaskHandle();
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    read_encoder(handle);
    xSemaphoreGive(handle->lock);
    handle->callback_task = NULL;
}

static void release_pins(gpio_num_t *pins, int count)
{
    for (int i = 0; i < count; i++)
    {
        gpio_set_pull_mode(pins[i], GPIO_FLOATING);
        gpio_set_direction(pins[i], GPIO_MODE_INPUT);
    }
}

esp_err_t rotary_encoder_create(const rotary_encoder_config_t *config, rotary_encoder_handle_t *handle)
{
    CHECK_ARG(handle && config && config->callback);
    CHECK_ARG(PIN_VALID(config->pin_a) && PIN_VALID(config->pin_b));
    CHECK_ARG(config->pin_a != config->pin_b);
    CHECK_ARG(!PIN_VALID(config->pin_btn)
              || (config->pin_btn != config->pin_a && config->pin_btn != config->pin_b));
    CHECK_ARG(config->btn_pressed_level <= 1);
    CHECK_ARG(config->acceleration_cap_ms > 0);
    CHECK_ARG(!config->acceleration_threshold_ms
              || config->acceleration_cap_ms <= config->acceleration_threshold_ms);

    *handle = NULL;

    rotary_encoder_handle_t re = calloc(1, sizeof(struct rotary_encoder));
    if (!re)
        return ESP_ERR_NO_MEM;

    re->pin_a = config->pin_a;
    re->pin_b = config->pin_b;
    re->pin_btn = config->pin_btn;
    re->btn_pressed_level = config->btn_pressed_level;
    re->btn_dead_time_us = config->btn_dead_time_us;
    re->btn_long_press_time_us = config->btn_long_press_time_us;
    re->acceleration_threshold_ms = config->acceleration_threshold_ms;
    re->acceleration_cap_ms = config->acceleration_cap_ms;
    re->callback = config->callback;
    re->callback_ctx = config->callback_ctx;

    re->lock = xSemaphoreCreateMutex();
    if (!re->lock)
    {
        free(re);
        return ESP_ERR_NO_MEM;
    }

    // Set up the GPIO pins as inputs. The pins are configured one by one, so
    // remember how many of them were taken, to release them if one of the
    // following calls fails and the pins would otherwise stay configured
    gpio_num_t pins[] = { re->pin_a, re->pin_b, re->pin_btn };
    const int num_pins = PIN_VALID(re->pin_btn) ? 3 : 2;
    int configured = 0;
    esp_err_t err = ESP_OK;

    for (int i = 0; i < num_pins && err == ESP_OK; i++)
    {
        err = gpio_set_direction(pins[i], GPIO_MODE_INPUT);
        if (err == ESP_OK && config->enable_internal_pullup)
            err = gpio_set_pull_mode(pins[i], GPIO_PULLUP_ONLY);
        if (err == ESP_OK)
            configured++;
    }

    if (err == ESP_OK)
    {
        const esp_timer_create_args_t timer_args =
        {
            .name = "__encoder__",
            .arg = re,
            .callback = encoder_timer_handler,
            .dispatch_method = ESP_TIMER_TASK
        };

        err = esp_timer_create(&timer_args, &re->timer);
        if (err == ESP_OK)
        {
            err = esp_timer_start_periodic(re->timer, RE_POLLING_INTERVAL_US);
            if (err != ESP_OK)
                esp_timer_delete(re->timer);
        }
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not create the encoder: %s", esp_err_to_name(err));
        release_pins(pins, configured);
        vSemaphoreDelete(re->lock);
        free(re);
        return err;
    }

    ESP_LOGI(TAG, "Created rotary encoder, A: %d, B: %d, BTN: %d", re->pin_a, re->pin_b, re->pin_btn);
    *handle = re;

    return ESP_OK;
}

esp_err_t rotary_encoder_delete(rotary_encoder_handle_t handle)
{
    CHECK_ARG(handle);

    // Deleting the encoder from its own callback would free the handle that
    // the callback is still running on
    if (is_callback_task(handle))
        return ESP_ERR_INVALID_STATE;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 1, 0)
    // Stopping the timer is not enough: the timer task dispatches the callback
    // after it releases its own lock, so the callback may have been dispatched
    // and still be waiting for our lock. The blocking stop waits for it
    esp_err_t err = esp_timer_stop_blocking(handle->timer, portMAX_DELAY);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Could not stop the timer: %s", esp_err_to_name(err));
        return err;
    }
#else
    // ESP-IDF before 6.1 cannot wait for the callback: the timer stops
    // dispatching new ones and the lock orders the callbacks that have already
    // started, but a callback that was dispatched and has not taken the lock
    // yet can still run after the handle is freed
    esp_timer_stop(handle->timer);
#endif

    esp_timer_delete(handle->timer);

    // Order the callbacks that have started on the older ESP-IDF versions
    xSemaphoreTake(handle->lock, portMAX_DELAY);
    xSemaphoreGive(handle->lock);
    vSemaphoreDelete(handle->lock);

    gpio_num_t pins[] = { handle->pin_a, handle->pin_b, handle->pin_btn };
    release_pins(pins, PIN_VALID(handle->pin_btn) ? 3 : 2);

    ESP_LOGI(TAG, "Deleted rotary encoder, A: %d, B: %d, BTN: %d", handle->pin_a, handle->pin_b, handle->pin_btn);
    free(handle);
    return ESP_OK;
}

esp_err_t rotary_encoder_enable_acceleration(rotary_encoder_handle_t handle, uint16_t coeff)
{
    CHECK_ARG(handle);

    // The callback holds the lock, so taking it here would deadlock
    if (is_callback_task(handle))
        return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    handle->acceleration.coeff = coeff;
    handle->acceleration.last_time = esp_timer_get_time();
    xSemaphoreGive(handle->lock);
    return ESP_OK;
}

esp_err_t rotary_encoder_disable_acceleration(rotary_encoder_handle_t handle)
{
    CHECK_ARG(handle);

    // The callback holds the lock, so taking it here would deadlock
    if (is_callback_task(handle))
        return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(handle->lock, portMAX_DELAY);
    handle->acceleration.coeff = 0;
    xSemaphoreGive(handle->lock);
    return ESP_OK;
}
