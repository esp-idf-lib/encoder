#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <encoder.h>

#include <esp_log.h>

#define EV_QUEUE_LEN 32

static const char *TAG = "encoder_example";

static QueueHandle_t event_queue;
static rotary_encoder_handle_t re;
static uint32_t dropped_events;

static void encoder_event_handler(const rotary_encoder_event_t *event, void *ctx)
{
    QueueHandle_t queue = (QueueHandle_t)ctx;

    // The callback runs in the timer task, so it cannot wait for the queue: the
    // events that do not fit have to be dropped, and the main loop reports them
    if (xQueueSendToBack(queue, event, 0) != pdTRUE)
        dropped_events++;
}

void test(void *arg)
{
    // Create queue for rotary encoder events
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));
    if (!event_queue)
    {
        ESP_LOGE(TAG, "Could not create the event queue");
        return;
    }

    // Create an encoder
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG();
    config.pin_a = (gpio_num_t)CONFIG_EXAMPLE_RE_A_GPIO;
    config.pin_b = (gpio_num_t)CONFIG_EXAMPLE_RE_B_GPIO;
    config.pin_btn = (gpio_num_t)CONFIG_EXAMPLE_RE_BTN_GPIO;
    config.callback = encoder_event_handler;
    config.callback_ctx = event_queue;
    ESP_ERROR_CHECK(rotary_encoder_create(&config, &re));

    rotary_encoder_event_t e;
    int32_t val = 0;

    ESP_LOGI(TAG, "Initial value: %" PRIi32, val);
    while (1)
    {
        xQueueReceive(event_queue, &e, portMAX_DELAY);

        if (dropped_events)
        {
            ESP_LOGW(TAG, "%" PRIu32 " event(s) were lost, the queue is too short", dropped_events);
            dropped_events = 0;
        }

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
