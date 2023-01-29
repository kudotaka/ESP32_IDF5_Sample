// https://akizukidenshi.com/download/ds/sitronix/st7032.pdf
// https://akizukidenshi.com/download/ds/xiamen/AQM0802A-FLW-GBW.pdf
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "i2c_device.h"

uint8_t St7032_ConvertHexNumber(uint8_t number);
void St7032_CreateOrignalCharacter(uint8_t address, uint8_t* pattern, uint8_t pattern_len);
void St7032_CursorByPotision(uint8_t row, uint8_t column);
void St7032_ShowDisplayByPotision(uint8_t row, uint8_t column, uint8_t data);
void St7032_ClearDisplay(void);
void St7032_ReturnHome(void);
void St7032_Display_OnOff(uint8_t nextState);
void St7032_Backlight_OnOff(uint8_t nextState);
esp_err_t St7032_WriteData(uint8_t t_data);
esp_err_t St7032_WriteCommand(uint8_t t_command);
#ifdef CONFIG_SOFTWARE_USE_BACKLIGHT
esp_err_t St7032_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud, gpio_num_t backlight);
#else
esp_err_t St7032_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud);
#endif

#ifdef __cplusplus
}
#endif
