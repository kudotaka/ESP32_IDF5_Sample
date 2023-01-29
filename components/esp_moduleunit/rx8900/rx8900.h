// https://akizukidenshi.com/catalog/g/gK-13009/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "time.h"
#include "driver/gpio.h"
#include "i2c_device.h"
#include "esp_unit_common.h"

esp_err_t Rx8900_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud);
void Rx8900_SetTime(rtc_date_t* data);
void Rx8900_GetTime(rtc_date_t* data);
void SetAlerm(rtc_date_t* data);

#ifdef __cplusplus
}
#endif
