// https://akizukidenshi.com/download/ds/sitronix/st7032.pdf
// https://akizukidenshi.com/download/ds/xiamen/AQM0802A-FLW-GBW.pdf
#include "freertos/FreeRTOS.h"
#include "st7032.h"

#define TAG "ST7032"

#define ST7032_ADDR (0x3E)

#define ST7032_COMMAND_CLEAR_DISPLAY         (0x01)
#define ST7032_COMMAND_RETURN_HOME           (0x02)
#define ST7032_COMMAND_ENTRYMODE_SHIFT_LEFT  (0x07)
#define ST7032_COMMAND_ENTRYMODE_SHIFT_RIGHT (0x05)
#define ST7032_COMMAND_DISPLAY_ONOFF_OFF     (0x08)
#define ST7032_COMMAND_DISPLAY_ONOFF_ON      (0x0C)
#define ST7032_COMMAND_SET_CGRAM_BASE        (0x40)
#define ST7032_COMMAND_SET_DDRAM_BASE        (0x80)

#define ST7032_DISPLAY_ROW_MIN    (1)
#define ST7032_DISPLAY_ROW_MAX    (2)
#define ST7032_DISPLAY_COLUMN_MIN (1)
#ifdef CONFIG_SOFTWARE_MODEL_AQM0802
#define ST7032_DISPLAY_COLUMN_MAX (8)
#define ST7032_CGRAM_POSITION_MAX (5)
#else
#define ST7032_DISPLAY_COLUMN_MAX (16)
#define ST7032_CGRAM_POSITION_MAX (7)
#endif

static I2CDevice_t st7032_device;
static gpio_num_t backlight_pin;

uint8_t St7032_ConvertHexNumber(uint8_t number) {
    if (number > 16) {
        return 0;
    }
    if (number > 9) {
        return 0x41 + number % 10;
    } else {
        return 0x30 + number;
    }
}

void St7032_CreateOrignalCharacter(uint8_t address, uint8_t* pattern, uint8_t pattern_len) {
    if (address > ST7032_CGRAM_POSITION_MAX) {
        ESP_LOGE(TAG, "Error arg. address:%d", address);
        return;
    }
    if (pattern_len > 8) {
        ESP_LOGE(TAG, "Error arg. pattern_len:%d", pattern_len);
        return;
    }

    St7032_WriteCommand(ST7032_COMMAND_SET_CGRAM_BASE | address << 3);
    vTaskDelay(pdMS_TO_TICKS(20));
    for (uint8_t i = 0; i < 8; i++) {
        St7032_WriteData(pattern[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    St7032_WriteCommand(ST7032_COMMAND_SET_DDRAM_BASE);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void St7032_CursorByPotision(uint8_t row, uint8_t column) {
    if (row > ST7032_DISPLAY_ROW_MAX || ST7032_DISPLAY_ROW_MIN > row
     || column > ST7032_DISPLAY_COLUMN_MAX || ST7032_DISPLAY_COLUMN_MIN > column) {
        ESP_LOGE(TAG, "Error arg. row:%d, column:%d", row, column);
        return;
    }

    switch (row)
    {
    case 1:
        St7032_WriteCommand(ST7032_COMMAND_SET_DDRAM_BASE + 0x00 + (column-1) );
        break;
    case 2:
        St7032_WriteCommand(ST7032_COMMAND_SET_DDRAM_BASE + 0x40 + (column-1) );
        break;    
    default:
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

void St7032_ShowDisplayByPotision(uint8_t row, uint8_t column, uint8_t data) {
    if (row > ST7032_DISPLAY_ROW_MAX || ST7032_DISPLAY_ROW_MIN > row
     || column > ST7032_DISPLAY_COLUMN_MAX || ST7032_DISPLAY_COLUMN_MIN > column) {
        ESP_LOGE(TAG, "Error arg. row:%d, column:%d", row, column);
        return;
    }

    St7032_CursorByPotision(row, column);
    vTaskDelay(pdMS_TO_TICKS(10));
    St7032_WriteData(data);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void St7032_ReturnHome() {
    St7032_WriteCommand(ST7032_COMMAND_RETURN_HOME);
    vTaskDelay(pdMS_TO_TICKS(20));
}

void St7032_ClearDisplay() {
    St7032_WriteCommand(ST7032_COMMAND_CLEAR_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(20));
}

void St7032_Display_OnOff(uint8_t nextState) {
    uint8_t value = nextState ? 1 : 0;
    if (value) {
        St7032_WriteCommand(ST7032_COMMAND_DISPLAY_ONOFF_ON);
    } else {
        St7032_WriteCommand(ST7032_COMMAND_DISPLAY_ONOFF_OFF);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
}

void St7032_Backlight_OnOff(uint8_t nextState) {
    uint8_t value = nextState ? 1 : 0;
    gpio_set_level(backlight_pin, value);
}

esp_err_t St7032_WriteData(uint8_t t_data) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {0};
    cmd[0] = 0x40;
    cmd[1] = t_data;
    ret = i2c_write_bytes(st7032_device, (uint32_t)I2C_NO_REG, cmd, (uint16_t)2);
    vTaskDelay(pdMS_TO_TICKS(1));
    return ret;
}

esp_err_t St7032_WriteCommand(uint8_t t_command) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {0};
    cmd[0] = 0x00;
    cmd[1] = t_command;
    ret = i2c_write_bytes(st7032_device, (uint32_t)I2C_NO_REG, cmd, (uint16_t)2);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ret;
}

#ifdef CONFIG_SOFTWARE_USE_BACKLIGHT
esp_err_t St7032_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud, gpio_num_t backlight) {
#else
esp_err_t St7032_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud) {
#endif
    esp_err_t ret = ESP_OK;
    st7032_device = i2c_malloc_device(i2c_num, sda, scl, baud, ST7032_ADDR);
    if (st7032_device == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
#ifdef CONFIG_SOFTWARE_USE_BACKLIGHT
    esp_err_t retpin = ESP_OK;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << backlight);

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    retpin = gpio_config(&io_conf);
    if (retpin != ESP_OK){
        ESP_LOGE(TAG, "Error configuring GPIO %d. Error code: 0x%x.", backlight, retpin);
    }
    backlight_pin = backlight;
    vTaskDelay(pdMS_TO_TICKS(100));
    St7032_Backlight_OnOff(true);
#endif

    vTaskDelay(pdMS_TO_TICKS(100));
    ret = St7032_WriteCommand(0x38);
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = St7032_WriteCommand(0x39);
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = St7032_WriteCommand(0x14);
    vTaskDelay(pdMS_TO_TICKS(20));
#ifdef CONFIG_SOFTWARE_MODEL_AQM0802
    ret = St7032_WriteCommand(0x70);
#else
    ret = St7032_WriteCommand(0x73);
#endif
    vTaskDelay(pdMS_TO_TICKS(20));
#ifdef CONFIG_SOFTWARE_MODEL_AQM0802
    ret = St7032_WriteCommand(0x56);
#else
    ret = St7032_WriteCommand(0x52);
#endif
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = St7032_WriteCommand(0x6C);
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = St7032_WriteCommand(0x38);
    vTaskDelay(pdMS_TO_TICKS(20));
#ifdef CONFIG_SOFTWARE_MODEL_AQM0802
    ret = St7032_WriteCommand(0x0C);
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = St7032_WriteCommand(0x01);
#else
    ret = St7032_WriteCommand(0x01);
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = St7032_WriteCommand(0x0C);
#endif
    vTaskDelay(pdMS_TO_TICKS(20));

    return ret;
}