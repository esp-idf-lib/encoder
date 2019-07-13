#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <encoder.h>

// Connect common encoder pin to ground

#define RE_A_GPIO   22
#define RE_B_GPIO   23
#define RE_BTN_GPIO 25

#define EV_QUEUE_LEN 5

static QueueHandle_t event_queue;
static rotary_encoder_t re;

void test(void *arg)
{
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    // Setup rotary encoder library
    ESP_ERROR_CHECK(rotary_encoder_init(event_queue));

    // Add one
    memset(&re, 0, sizeof(rotary_encoder_t));
    re.pin_a = RE_A_GPIO;
    re.pin_b = RE_B_GPIO;
    re.pin_btn = RE_BTN_GPIO;
    ESP_ERROR_CHECK(rotary_encoder_add(&re));

    rotary_encoder_event_t e;
    int32_t vol = 0;

    printf("Initial volume level: %d\n", vol);

    while (1)
    {
        xQueueReceive(event_queue, &e, portMAX_DELAY);

        printf("Got encoder event. type = %d, sender = 0x%08x, diff = %d\n", e.type, (uint32_t)e.sender, e.diff);

        switch (e.type)
        {
            case RE_ET_BTN_PRESSED:
                printf("Button was pressed\n");
                break;
            case RE_ET_BTN_RELEASED:
                printf("Button was released\n");
                break;
            case RE_ET_BTN_LONG_PRESSED:
                printf("Looooong pressed button\n");
                break;
            case RE_ET_CHANGED:
                if (e.diff < 0)
                    vol--;
                else if (e.diff > 0)
                    vol++;
                printf("Volume was changed to %d\n", vol);
                break;
            default:
                break;
        }
    }
}

void app_main()
{
    xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
