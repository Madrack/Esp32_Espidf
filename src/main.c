/*
 * Copyright (c) 2019 David Antliff
 *
 * This program provides an example using the esp32-rotary-encoder component.
 * Events are received via an event queue and displayed on the serial console.
 * The task also polls the device position every second to show that the latest
 * event always matches the current position.
 *
 * esp32-rotary-encoder is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * esp32-rotary-encoder is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with esp32-rotary-encoder.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "rotary_encoder.h"

#define TAG "app"

#define ROT_ENC_A_GPIO (16)
#define ROT_ENC_B_GPIO (17)

#define ENABLE_HALF_STEPS true  // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT          0      // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION    false  // Set to true to reverse the clockwise/counterclockwise sense

void taskEncoder(void *pvParametr) {
    // esp32-rotary-encoder requires that the GPIO ISR service is installed before calling rotary_encoder_register()
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Initialise the rotary encoder device with the GPIOs for A and B signals
    rotary_encoder_info_t info = { 0 };
    ESP_ERROR_CHECK(rotary_encoder_init(&info, ROT_ENC_A_GPIO, ROT_ENC_B_GPIO));
    ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS));
#ifdef FLIP_DIRECTION
    ESP_ERROR_CHECK(rotary_encoder_flip_direction(&info));
#endif

    // Create a queue for events from the rotary encoder driver.
    // Tasks can read from this queue to receive up to date position information.
    QueueHandle_t event_queue = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue));
    while (1)
    {
        // Wait for incoming events on the event queue.
        rotary_encoder_event_t event = { 0 };
        if (xQueueReceive(event_queue, &event, 1000 / portTICK_PERIOD_MS) == pdTRUE)
        {
            ESP_LOGI(TAG, "Event: position %d, direction %s", event.state.position,
                     event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");
        }
        else
        {
            // Poll current position and direction
            rotary_encoder_state_t state = { 0 };
            ESP_ERROR_CHECK(rotary_encoder_get_state(&info, &state));
            ESP_LOGI(TAG, "Poll: position %d, direction %s", state.position,
                     state.direction ? (state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");

            // Reset the device
            if (RESET_AT && (state.position >= RESET_AT || state.position <= -RESET_AT))
            {
                ESP_LOGI(TAG, "Reset");
                ESP_ERROR_CHECK(rotary_encoder_reset(&info));
            }
        }
    }
    ESP_LOGE(TAG, "queue receive failed");
    ESP_ERROR_CHECK(rotary_encoder_uninit(&info));
    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(taskEncoder, "taskEncoder", 4096, NULL, 5, NULL);
}




// #include "sdkconfig.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "esp_log.h"
// #include "driver/pulse_cnt.h"
// #include "driver/gpio.h"
// #include "esp_sleep.h"

// static const char *TAG = "example";

// #define EXAMPLE_PCNT_HIGH_LIMIT 100
// #define EXAMPLE_PCNT_LOW_LIMIT  -100

// #define EXAMPLE_EC11_GPIO_A 16
// #define EXAMPLE_EC11_GPIO_B 17

// static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
// {
//     BaseType_t high_task_wakeup;
//     QueueHandle_t queue = (QueueHandle_t)user_ctx;
//     // send event data to queue, from this interrupt callback
//     xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
//     return (high_task_wakeup == pdTRUE);
// }

// void app_main(void)
// {
//     ESP_LOGI(TAG, "install pcnt unit");
//     pcnt_unit_config_t unit_config = {
//         .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
//         .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
//     };
//     pcnt_unit_handle_t pcnt_unit = NULL;
//     ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

//     ESP_LOGI(TAG, "set glitch filter");
//     pcnt_glitch_filter_config_t filter_config = {
//         .max_glitch_ns = 1000,
//     };
//     ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

//     ESP_LOGI(TAG, "install pcnt channels");
//     pcnt_chan_config_t chan_a_config = {
//         .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
//         .level_gpio_num = EXAMPLE_EC11_GPIO_B,
//     };
//     pcnt_channel_handle_t pcnt_chan_a = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
//     pcnt_chan_config_t chan_b_config = {
//         .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
//         .level_gpio_num = EXAMPLE_EC11_GPIO_A,
//     };
//     pcnt_channel_handle_t pcnt_chan_b = NULL;
//     ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

//     ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
//     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
//     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
//     ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
//     ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

//     ESP_LOGI(TAG, "add watch points and register callbacks");
//     int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -1, 0, 1, EXAMPLE_PCNT_HIGH_LIMIT};
//     // int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, 10, 0, 10, EXAMPLE_PCNT_HIGH_LIMIT};
//     for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
//         ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
//     }
//     pcnt_event_callbacks_t cbs = {
//         .on_reach = example_pcnt_on_reach,
//     };
//     QueueHandle_t queue = xQueueCreate(10, sizeof(int));
//     ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

//     ESP_LOGI(TAG, "enable pcnt unit");
//     ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
//     ESP_LOGI(TAG, "clear pcnt unit");
//     ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
//     ESP_LOGI(TAG, "start pcnt unit");
//     ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

// #if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
//     // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
//     ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
//     ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
//     ESP_ERROR_CHECK(esp_light_sleep_start());
// #endif

//     // Report counter value
//     int pulse_count = 0;
//     int event_count = 0;
//     while (1) {
//         if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
//             ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
//         } else {
//             // ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
//             // ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
//         }
//     }
// }

// #include <inttypes.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <string.h>
// //#include <encoder.h>
// #include "encoder.h"
// #include <esp_idf_lib_helpers.h>
// #include <esp_log.h>

// // Connect common encoder pin to ground
// #if HELPER_TARGET_IS_ESP8266
// #define RE_A_GPIO   14
// #define RE_B_GPIO   12
// #define RE_BTN_GPIO 13

// #elif HELPER_TARGET_IS_ESP32
// #define RE_A_GPIO   16
// #define RE_B_GPIO   17
// #define RE_BTN_GPIO 5

// #else
// #error Unknown platform
// #endif

// #define EV_QUEUE_LEN 5

// static const char *TAG = "encoder_example";

// static QueueHandle_t event_queue;
// static rotary_encoder_t re;

// void test(void *arg)
// {
//     // Create event queue for rotary encoders
//     event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

//     // Setup rotary encoder library
//     ESP_ERROR_CHECK(rotary_encoder_init(event_queue));

//     // Add one encoder
//     memset(&re, 0, sizeof(rotary_encoder_t));
//     re.pin_a = RE_A_GPIO;
//     re.pin_b = RE_B_GPIO;
//     re.pin_btn = RE_BTN_GPIO;
//     ESP_ERROR_CHECK(rotary_encoder_add(&re));

//     rotary_encoder_event_t e;
//     int32_t val = 0;

//     ESP_LOGI(TAG, "Initial value: %" PRIi32, val);
//     while (1)
//     {
//         xQueueReceive(event_queue, &e, portMAX_DELAY);

//         switch (e.type)
//         {
//             case RE_ET_BTN_PRESSED:
//                 ESP_LOGI(TAG, "Button pressed");
//                 break;
//             case RE_ET_BTN_RELEASED:
//                 ESP_LOGI(TAG, "Button released");
//                 break;
//             case RE_ET_BTN_CLICKED:
//                 ESP_LOGI(TAG, "Button clicked");
//                 break;
//             case RE_ET_BTN_LONG_PRESSED:
//                 ESP_LOGI(TAG, "Looooong pressed button");
//                 break;
//             case RE_ET_CHANGED:
//                 val += e.diff;
//                 ESP_LOGI(TAG, "Value = %" PRIi32, val);
//                 break;
//             default:
//                 break;
//         }
//     }
// }

// void app_main()
// {
//     xTaskCreate(test, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
// } 



/* #include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// task 1
void task1(void *pvParametr) {
    while(1) {
        ESP_LOGI("TASK1", "Task 1 executed");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    vTaskDelete(NULL);
}

void task2(void *pvParametr) {
    while(1) {
        ESP_LOGI("TASK1", "Task 2 executed");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}

void app_main() {
    xTaskCreate(task1, "task1", 4096, NULL, 5, NULL);
    xTaskCreate(task2, "task2", 4096, NULL, 5, NULL);
} */