// https://github.com/m5stack/M5Unit-ENV/blob/master/src/QMP6988.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "i2c_device.h"

esp_err_t Qmp6988_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud);
float Qmp6988_CalcAltitude(float pressure, float temp);
float Qmp6988_CalcPressure();
float Qmp6988_CalcTemperature();
float Qmp6988_GetPressure();
float Qmp6988_GetTemperature();

void Qmp6988_SetpPowermode(int power_mode);
void Qmp6988_SetFilter(uint8_t filter);
void Qmp6988_SetOversamplingP(uint8_t oversampling_p);
void Qmp6988_SetOversamplingT(uint8_t oversampling_t);

#ifdef __cplusplus
}
#endif
