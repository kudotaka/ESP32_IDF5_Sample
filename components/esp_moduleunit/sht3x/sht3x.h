// https://github.com/m5stack/M5Unit-ENV/blob/master/src/SHT3X.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "i2c_device.h"

esp_err_t Sht3x_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud);
esp_err_t Sht3x_Read();
float Sht3x_GetTemperature();
int32_t Sht3x_GetIntTemperature();
float Sht3x_GetHumidity();
int32_t Sht3x_GetIntHumidity();

#ifdef __cplusplus
}
#endif
