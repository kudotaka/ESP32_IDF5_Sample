// M5Atom Hardware Button.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "button.h"

#define TAG "BUTTON"

Button_t* button_ahead = NULL;
static SemaphoreHandle_t button_lock = NULL;
static void Button_UpdateTask(void *arg);

void Button_Init() {
    if (button_lock == NULL) {
        button_lock = xSemaphoreCreateMutex();
        xTaskCreatePinnedToCore(Button_UpdateTask, "Button_UpdateTask", 2 * 1024, NULL, 1, NULL, 0);
    }
}

esp_err_t Button_Enable(gpio_num_t pin, ButtonActiveType type) {
    esp_err_t ret = ESP_OK;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << pin);

    io_conf.mode = GPIO_MODE_INPUT;
    if (type == BUTTON_ACTIVE_LOW) {
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

Button_t* Button_Attach(gpio_num_t pin, ButtonActiveType type) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    Button_t *button = (Button_t *)malloc(sizeof(Button_t) * 1);
    if (button != NULL) {
        button->pin = pin;
        button->type = type;
        button->last_value = 0;
        button->last_press_time = 0;
        button->long_press_time = 0;
        button->next = NULL;
        button->state = 0;
        if (button_ahead == NULL) {
            button_ahead = button;
        } else {
            Button_t* button_last = button_ahead;
            while (button_last->next != NULL) {
                button_last = button_last->next;
            }
            button_last->next = button;
        }
    }
    xSemaphoreGive(button_lock);
    return button;
}

bool Button_Read(gpio_num_t pin) {
    //- 0 the GPIO input level is 0
    //- 1 the GPIO input level is 1
    return gpio_get_level(pin);
}

uint8_t Button_WasPressed(Button_t* button) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    uint8_t result = (button->state & PRESS) > 0;
    button->state &= ~PRESS;
    xSemaphoreGive(button_lock);
    return result;
}

uint8_t Button_WasReleased(Button_t* button) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    uint8_t result = (button->state & RELEASE) > 0;
    button->state &= ~RELEASE;
    xSemaphoreGive(button_lock);
    return result;
}

uint8_t Button_WasLongPress(Button_t* button, uint32_t long_press_time) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    button->long_press_time = long_press_time;
    uint8_t result = (button->state & LONGPRESS) > 0;
    button->state &= ~LONGPRESS;
    xSemaphoreGive(button_lock);
    return result;
}

uint8_t Button_IsPress(Button_t* button) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    uint8_t result = (button->value == 1);
    xSemaphoreGive(button_lock);
    return result;
}

uint8_t Button_IsRelease(Button_t* button) {
    xSemaphoreTake(button_lock, portMAX_DELAY);
    uint8_t result = (button->value == 0);
    xSemaphoreGive(button_lock);
    return result;
}

void Button_Update(Button_t* button, uint8_t press) {
    uint8_t value = press;
    uint32_t now_ticks = xTaskGetTickCount();
    if (value != button->last_value) {
        if (value == 1) {
            button->state |= PRESS;
            button->last_press_time = now_ticks;
        } else {
            if (button->long_press_time && (now_ticks - button->last_press_time > button->long_press_time)) {
                button->state |= LONGPRESS;
            } else {
                button->state |= RELEASE;
            }
        }
        button->last_value = value;
    }
    button->last_value = value;
    button->value = value;
}

static void Button_UpdateTask(void *arg) {
    Button_t* button;

    for (;;) {
        xSemaphoreTake(button_lock, portMAX_DELAY);
        button = button_ahead;
        while (button != NULL) {
            if (button->type == BUTTON_ACTIVE_LOW) {
                Button_Update(button, !Button_Read(button->pin));
            } else {
                Button_Update(button, Button_Read(button->pin));
            }
            button = button->next;
        }
        xSemaphoreGive(button_lock);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}