#ifdef CONFIG_SOFTWARE_ESP_LCD_UI_SUPPORT
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "lvgl.h"


#ifdef CONFIG_SOFTWARE_UNIT_RTC_SUPPORT
void ui_datetime_set(char *dateTxt);
#endif

#ifdef CONFIG_SOFTWARE_UNIT_ENV2_SUPPORT
void ui_temperature_update(int32_t value);
void ui_humidity_update(int32_t value);
#endif

#ifdef CONFIG_SOFTWARE_UNIT_BUTTON_SUPPORT
void ui_button_label_update(bool state);
#endif

#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
void ui_wifi_label_update(bool state);
void ui_start();
void Ssd1306_I2c_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, gpio_num_t rst, uint32_t baud);
#endif

#endif