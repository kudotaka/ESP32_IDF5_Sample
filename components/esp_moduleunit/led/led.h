#pragma once

#include "stdio.h"
#include "driver/gpio.h"
#include "esp_log.h"


typedef enum _LedActiveType{
    LED_ACTIVE_LOW = 0,
    LED_ACTIVE_HIGH = 1,
} LedActiveType;

typedef struct _Led_t  {
    gpio_num_t pin;             //GPIO
    LedActiveType type;
    uint8_t state;              //The led state
} Led_t;

void Led_Init();
esp_err_t Led_Enable(gpio_num_t pin, LedActiveType type);
Led_t* Led_Attach(gpio_num_t pin, LedActiveType type);
esp_err_t Led_OnOff(Led_t* led, uint8_t nextState);
