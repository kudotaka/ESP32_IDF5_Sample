// https://github.com/Seeed-Studio/Grove_BMP280/blob/master/Seeed_BMP280.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "i2c_device.h"

esp_err_t Bmp280_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud);
float Bmp280_CalcAltitude(float p0);
float Bmp280_GetPressure(void);
float Bmp280_GetTemperature(void);

// private functoins
uint8_t Bmp280_Read8(uint8_t reg);
uint16_t Bmp280_Read16(uint8_t reg);
uint16_t Bmp280_Read16LE(uint8_t reg);
int16_t Bmp280_ReadS16(uint8_t reg);
int16_t Bmp280_ReadS16LE(uint8_t reg);
uint32_t Bmp280_Read24(uint8_t reg);

#ifdef __cplusplus
}
#endif
