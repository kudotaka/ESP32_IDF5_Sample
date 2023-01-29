/**
 * @file pcf8563.h
 * @brief Functions for the PCF8563 Real-Time Clock (RTC).
 */

#pragma once
#include "stdint.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "i2c_device.h"
#include "esp_unit_common.h"

esp_err_t PCF8563_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud);
void PCF8563_SetTime(rtc_date_t* data);
void PCF8563_GetTime(rtc_date_t* data);
void PCF8563_SetAlarmIRQ(int8_t minute, int8_t hour, int8_t day, int8_t week);
int16_t PCF8563_SetTimerIRQ(int16_t value);
int16_t PCF8563_GetTimerTime();
uint8_t PCF8563_GetIRQ();
void PCF8563_ClearIRQ();
