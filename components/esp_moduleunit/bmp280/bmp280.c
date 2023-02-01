// https://github.com/Seeed-Studio/Grove_BMP280/blob/master/Seeed_BMP280.cpp
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "math.h"
#include "bmp280.h"

#define BMP280_ADDR   (0x76)

#define BMP280_REG_DIG_T1    0x88
#define BMP280_REG_DIG_T2    0x8A
#define BMP280_REG_DIG_T3    0x8C

#define BMP280_REG_DIG_P1    0x8E
#define BMP280_REG_DIG_P2    0x90
#define BMP280_REG_DIG_P3    0x92
#define BMP280_REG_DIG_P4    0x94
#define BMP280_REG_DIG_P5    0x96
#define BMP280_REG_DIG_P6    0x98
#define BMP280_REG_DIG_P7    0x9A
#define BMP280_REG_DIG_P8    0x9C
#define BMP280_REG_DIG_P9    0x9E

#define BMP280_REG_CHIPID          0xD0
#define BMP280_REG_VERSION         0xD1
#define BMP280_REG_SOFTRESET       0xE0

#define BMP280_REG_CONTROL         0xF4
#define BMP280_REG_CONFIG          0xF5
#define BMP280_REG_PRESSUREDATA    0xF7
#define BMP280_REG_TEMPDATA        0xFA

static const char *TAG = "BMP280";

static I2CDevice_t bmp280_device;
bool isTransport_OK;

// Calibratino data
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int32_t t_fine;

uint8_t Bmp280_Read8(uint8_t reg) {
    esp_err_t ret;
    uint8_t data[1];
    ret = i2c_read_bytes(bmp280_device, reg, data, 1);
    if (ret != ESP_OK) {
        isTransport_OK = false;
        return 0;
    } else {
        isTransport_OK = true;
    }
    return data[0];
}

uint16_t Bmp280_Read16(uint8_t reg) {
    esp_err_t ret;
    uint8_t data[2];
    ret = i2c_read_bytes(bmp280_device, reg, data, 2);
    if (ret != ESP_OK) {
        isTransport_OK = false;
        return 0;
    } else {
        isTransport_OK = true;
    }
    return (uint16_t) data[0] << 8 | data[1];
}

uint16_t Bmp280_Read16LE(uint8_t reg) {
  uint16_t data = Bmp280_Read16(reg);
  return (data >> 8) | (data << 8);
}

int16_t Bmp280_ReadS16(uint8_t reg) {
  return (int16_t)Bmp280_Read16(reg);
}

int16_t Bmp280_ReadS16LE(uint8_t reg) {
  return (int16_t)Bmp280_Read16LE(reg);
}

uint32_t Bmp280_Read24(uint8_t reg) {
    esp_err_t ret;
    uint8_t data[3];
    ret = i2c_read_bytes(bmp280_device, reg, data, 3);
    if (ret != ESP_OK) {
        isTransport_OK = false;
        return 0;
    } else {
        isTransport_OK = true;
    }
    return (uint32_t) data[0] << 16 | data[1] << 8 | data[2];
}


float Bmp280_GetTemperature(void) {
  int32_t var1, var2;
  int32_t adc_T = Bmp280_Read24(BMP280_REG_TEMPDATA);
  // Check if the last transport successed
  if (!isTransport_OK) {
    ESP_LOGE(TAG, "Bmp280_GetTemperature error!");
    return 0;
  }
  adc_T >>= 4;
  var1 = (((adc_T >> 3) - ((int32_t)(dig_T1 << 1))) *
          ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
            ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
          ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

float Bmp280_GetPressure(void) {
  int64_t var1, var2, p;
  // Call getTemperature to get t_fine
  Bmp280_GetTemperature();
  // Check if the last transport successed
  if (!isTransport_OK) {
    ESP_LOGE(TAG, "Bmp280_GetPressure error!");
    return 0;
  }
  int32_t adc_P = Bmp280_Read24(BMP280_REG_PRESSUREDATA);
  adc_P >>= 4;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return p / 256;
}

float Bmp280_CalcAltitude_3(float p0, float p1, float t) {
  float C;
  C = (p0 / p1);
  C = pow(C, (1 / 5.25588)) - 1.0;
  C = (C * (t + 273.15)) / 0.0065;
  return C;
}

float Bmp280_CalcAltitude(float p0) {
  if (!isTransport_OK) {
    ESP_LOGE(TAG, "Bmp280_CalcAltitude error!");
    return 0;
  }
  float t = Bmp280_GetTemperature();
  float p1 = Bmp280_GetPressure();
  return Bmp280_CalcAltitude_3(p0, p1, t);
}

esp_err_t Bmp280_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud) {
    bmp280_device = i2c_malloc_device(i2c_num, sda, scl, baud, BMP280_ADDR);
    if (bmp280_device == NULL) {
        ESP_LOGE(TAG, "Bmp280_Init error!");
        return ESP_ERR_NOT_FOUND;
    }

    dig_T1 = Bmp280_Read16LE(BMP280_REG_DIG_T1);
    dig_T2 = Bmp280_ReadS16LE(BMP280_REG_DIG_T2);
    dig_T3 = Bmp280_ReadS16LE(BMP280_REG_DIG_T3);
    dig_P1 = Bmp280_Read16LE(BMP280_REG_DIG_P1);
    dig_P2 = Bmp280_ReadS16LE(BMP280_REG_DIG_P2);
    dig_P3 = Bmp280_ReadS16LE(BMP280_REG_DIG_P3);
    dig_P4 = Bmp280_ReadS16LE(BMP280_REG_DIG_P4);
    dig_P5 = Bmp280_ReadS16LE(BMP280_REG_DIG_P5);
    dig_P6 = Bmp280_ReadS16LE(BMP280_REG_DIG_P6);
    dig_P7 = Bmp280_ReadS16LE(BMP280_REG_DIG_P7);
    dig_P8 = Bmp280_ReadS16LE(BMP280_REG_DIG_P8);
    dig_P9 = Bmp280_ReadS16LE(BMP280_REG_DIG_P9);
    i2c_write_byte(bmp280_device, BMP280_REG_CONTROL, 0x3F);

    return ESP_OK; 
}