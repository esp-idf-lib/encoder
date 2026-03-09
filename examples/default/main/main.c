#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <encoder.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>

// Connect common encoder pin to ground
#if HELPER_TARGET_IS_ESP8266
#define RE_A_GPIO   14
#define RE_B_GPIO   12
#define RE_BTN_GPIO 13

#elif HELPER_TARGET_IS_ESP32
#define RE_A_GPIO   16
#define RE_B_GPIO   17
#define RE_BTN_GPIO 5

#else
#error Unknown platform
#endif

#define EV_QUEUE_LEN 5

static const char *TAG = "encoder_example";

static QueueHandle_t event_queue;
static rotary_encoder_handle_t re;

static void encoder_event_handler(const rotary_encoder_event_t *event, void *ctx)
{
    QueueHandle_t queue = (QueueHandle_t)ctx;
    xQueueSendToBack(queue, event, 0);
}

void test(void *arg)
{
    // Create queue for rotary encoder events
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    // Create an encoder
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG();
    config.pin_a = RE_A_GPIO;
    config.pin_b = RE_B_GPIO;
    config.pin_btn = RE_BTN_GPIO;
    config.callback = encoder_event_handler;
    config.callback_ctx = event_queue;
    ESP_ERROR_CHECK(rotary_encoder_create(&config, &re));

    rotary_encoder_event_t e;
    int32_t val = 0;

    ESP_LOGI(TAG, "Initial value: %" PRIi32, val);
    while (1)
    {
        xQueueReceive(event_queue, &e, portMAX_DELAY);

        switch (e.type)
        {
            case RE_ET_BTN_PRESSED:
                ESP_LOGI(TAG, "Button pressed");
                break;
            case RE_ET_BTN_RELEASED:
                ESP_LOGI(TAG, "Button released");
                break;
            case RE_ET_BTN_CLICKED:
                ESP_LOGI(TAG, "Button clicked");
                rotary_encoder_enable_acceleration(re, 100);
                ESP_LOGI(TAG, "Acceleration enabled");
                break;
            case RE_ET_BTN_LONG_PRESSED:
                ESP_LOGI(TAG, "Looooong pressed button");
                rotary_encoder_disable_acceleration(re);
                ESP_LOGI(TAG, "Acceleration disabled");
                break;
            case RE_ET_CHANGED:
                val += e.diff;
                ESP_LOGI(TAG, "Value = %" PRIi32, val);
                break;
            default:
                break;
        }
    }
}

void app_main()
{
    xTaskCreate(test, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
