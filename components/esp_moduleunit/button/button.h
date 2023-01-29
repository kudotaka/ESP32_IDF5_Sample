#pragma once

#include "stdio.h"
#include "driver/gpio.h"
#include "esp_log.h"

typedef enum {
    PRESS = (1 << 0),       //button was pressed.
    RELEASE = (1 << 1),     //button was released.
    LONGPRESS = (1 << 2),   //button was long pressed.
} PressEvent;

typedef enum _ButtonActiveType{
    BUTTON_ACTIVE_LOW = 0,
    BUTTON_ACTIVE_HIGH = 1,
} ButtonActiveType;

typedef struct _Button_t  {
    gpio_num_t pin;             //GPIO
    ButtonActiveType type;
    uint8_t value;              //Current button touched state
    uint8_t last_value;         //Previous button touched state
    uint32_t last_press_time;   //FreeRTOS ticks when button was last touched
    uint32_t long_press_time;   //Number of FreeRTOS ticks to elapse to consider holding the button a long press
    uint8_t state;              //The button press event
    struct _Button_t* next;     //Pointer to the next button
} Button_t;

void Button_Init();
esp_err_t Button_Enable(gpio_num_t pin, ButtonActiveType type);
Button_t* Button_Attach(gpio_num_t pin, ButtonActiveType type);
uint8_t Button_WasPressed(Button_t* button);
uint8_t Button_WasReleased(Button_t* button);
uint8_t Button_IsPress(Button_t* button);
uint8_t Button_IsRelease(Button_t* button);
uint8_t Button_WasLongPress(Button_t* button, uint32_t long_press_time);
