// M5StickCPlus Internal LED.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "led.h"

#define TAG "LED"

static SemaphoreHandle_t led_lock = NULL;

void Led_Init() {
    if (led_lock == NULL) {
        led_lock = xSemaphoreCreateMutex();
    }
}

esp_err_t Led_Enable(gpio_num_t pin, LedActiveType type) {
    esp_err_t ret = ESP_OK;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << pin);

    io_conf.mode = GPIO_MODE_OUTPUT;
    if (type == LED_ACTIVE_LOW) {
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    } else {
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    }
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Error configuring GPIO %d. Error code: 0x%x.", pin, ret);
    }
    return ret;
}

Led_t* Led_Attach(gpio_num_t pin, LedActiveType type) {
    xSemaphoreTake(led_lock, portMAX_DELAY);
    Led_t *led = (Led_t *)malloc(sizeof(Led_t) * 1);
    led->pin = pin;
    led->type = type;
    led->state = 0;
    xSemaphoreGive(led_lock);
    return led;
}

esp_err_t Led_OnOff(Led_t* led, uint8_t nextState) {
    xSemaphoreTake(led_lock, portMAX_DELAY);
    uint8_t value = 0;
    if (led->type == LED_ACTIVE_LOW) {
        value = nextState ? 0 : 1;
    } else {
        value = nextState ? 1 : 0;
    }
    esp_err_t err = gpio_set_level(led->pin, value);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Error setting GPIO %d state. Error code: 0x%x.", led->pin, err);
    }
    led->state = value;
    xSemaphoreGive(led_lock);
    return err;
}
