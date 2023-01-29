// https://github.com/Seeed-Studio/Grove_4Digital_Display
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "driver/gpio.h"

typedef struct _DigitDisplay_t  {
    gpio_num_t clk;
    gpio_num_t data;
    uint8_t brightness;
} DigitDisplay_t;

/*******************Definitions for TM1637*********************/
#define ADDR_AUTO 0x40
#define ADDR_FIXED 0x44

#define STARTADDR 0xc0
#define DATA_CREAR 0x7f
/*****Definitions for the clock point of the digit tube *******/
#define POINT_ON 1
#define POINT_OFF 0
/**************Definitions for brightness**********************/
#define BRIGHT_DARKEST 0   // OSL10564-IG
#define BRIGHT_TYPICAL 2   // OSL10564-IB
#define BRIGHT_BRIGHTEST 7 // OSL10564-IYG

#define CMD_DISP_CTRL_BASE 0x88

void Tm1637_Init();
esp_err_t Tm1637_Enable(gpio_num_t clk, gpio_num_t data);
DigitDisplay_t* Tm1637_Attach(gpio_num_t clk, gpio_num_t data, uint8_t brightness);
esp_err_t Tm1637_PinMode(gpio_num_t pin, gpio_mode_t mode);
uint8_t Tm1637_PinRead(gpio_num_t pin);
esp_err_t Tm1637_PinWrite(gpio_num_t pin, bool level);
uint8_t Tm1637_WriteByte(DigitDisplay_t* led, int8_t wr_data);
void Tm1637_Start(DigitDisplay_t* digitdisplay);
void Tm1637_Stop(DigitDisplay_t* digitdisplay);
void Tm1637_DisplayAll(DigitDisplay_t* digitdisplay, uint8_t disp_data[]);
void Tm1637_DisplayBit(DigitDisplay_t* digitdisplay, uint8_t bit_addr, uint8_t disp_data);
void Tm1637_DisplayBitAddPoint(DigitDisplay_t* digitdisplay, uint8_t bit_addr, uint8_t disp_data, uint8_t add_point);
void Tm1637_DisplayStr(DigitDisplay_t* digitdisplay, char str[], uint16_t loop_delay);
void Tm1637_DisplayBitRowdata(DigitDisplay_t* digitdisplay, uint8_t bit_addr, uint8_t disp_data);
void Tm1637_ClearDisplay(DigitDisplay_t* digitdisplay);
void Tm1637_Coding_Full(uint8_t disp_data[]);
uint8_t Tm1637_Coding_One(uint8_t disp_data, uint8_t add_point);
void Tm1637_BitDelay(void);

#ifdef __cplusplus
}
#endif