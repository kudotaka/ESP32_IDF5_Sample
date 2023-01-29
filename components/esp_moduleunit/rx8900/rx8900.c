// https://akizukidenshi.com/catalog/g/gK-13009/
#include "freertos/FreeRTOS.h"
#include "rx8900.h"

#define RX8900_ADDR (0x32)

#define RX8900_REG_SEC         (0x00)
#define RX8900_REG_MIN         (0x01)
#define RX8900_REG_HOUR        (0x02)
#define RX8900_REG_WEEK        (0x03)
#define RX8900_REG_DAY         (0x04)
#define RX8900_REG_MONTH       (0x05)
#define RX8900_REG_YEAR        (0x06)
#define RX8900_REG_RAM         (0x07)
#define RX8900_REG_MIN_ALARM   (0x08)
#define RX8900_REG_HOUR_ALARM  (0x09)
#define RX8900_REG_WEEK_DAY_ALARM  (0x0A)
#define RX8900_REG_TIMER_COUNTER_0 (0x0B)
#define RX8900_REG_TIMER_COUNTER_1 (0x0C)
#define RX8900_REG_EXTENSION_REGISTER (0x0D)
#define RX8900_REG_FLAG_REGSTER       (0x0E)
#define RX8900_REG_CONTROL_REGSTER    (0x0F)
#define RX8900_REG_TEMP               (0x17)
#define RX8900_REG_BACKUP_FUNCTION    (0x18)

static I2CDevice_t rx8900_device;

static void I2CWrite(uint8_t addr, uint8_t* buf, uint8_t len) {
    i2c_write_bytes(rx8900_device, addr, buf, len);
}

static void I2CWriteByte(uint8_t addr, uint8_t data) {
    i2c_write_byte(rx8900_device, addr, data);
}

static void I2CRead(uint8_t addr, uint8_t* buf, uint8_t len) {
    i2c_read_bytes_no_stop(rx8900_device, addr, buf, len);
}

static uint8_t byte2BCD(uint8_t data) {
    return ((data / 10) << 4) + data % 10;
}

static uint8_t BCD2Byte(uint8_t data) {
    return (data >> 4) * 10 + (data & 0x0f);
}

uint8_t subZeller( uint16_t y, uint16_t m, uint16_t d ) {
  if( m < 3 ) {
    y--; m += 12;
  }
  return ( y + y/4 - y/100 + y/400 + ( 13*m + 8 )/5 + d )%7;
}

void Reset() {
    uint8_t time_buf[1];
    I2CRead(RX8900_REG_CONTROL_REGSTER, time_buf, 1);
    I2CWriteByte(RX8900_REG_CONTROL_REGSTER, time_buf[1] | 0b00000001 );
}

void Reset_WADA() {
    uint8_t time_buf[1];
    I2CRead(RX8900_REG_EXTENSION_REGISTER, time_buf, 1);
    I2CWriteByte(RX8900_REG_EXTENSION_REGISTER, time_buf[1] & 0b10111111 );
}

void Set_WADA() {
    uint8_t time_buf[1];
    I2CRead(RX8900_REG_EXTENSION_REGISTER, time_buf, 1);
    I2CWriteByte(RX8900_REG_EXTENSION_REGISTER, time_buf[1] | 0b10111111 );
}

void Set_AIE() {
    uint8_t time_buf[1];
    I2CRead(RX8900_REG_CONTROL_REGSTER, time_buf, 1);
    I2CWriteByte(RX8900_REG_CONTROL_REGSTER, time_buf[1] | 0b00001000 );
}

void Reset_AIE() {
    uint8_t time_buf[1];
    I2CRead(RX8900_REG_CONTROL_REGSTER, time_buf, 1);
    I2CWriteByte(RX8900_REG_CONTROL_REGSTER, time_buf[1] & 0b00001000 );
}

void Reset_AF() {
    uint8_t time_buf[1];
    I2CRead(RX8900_REG_FLAG_REGSTER, time_buf, 1);
    I2CWriteByte(RX8900_REG_FLAG_REGSTER, time_buf[1] & 0b11110111 );
}
bool Is_AF() {
    uint8_t time_buf[1];
    I2CRead(RX8900_REG_FLAG_REGSTER, time_buf, 1);
    return (time_buf[1] & 0b00001000) ? true : false;
}

void SetAlerm(rtc_date_t* data) {
    if (data == NULL) {
        return ;
    }

    Set_AIE();
    Set_WADA();
    uint8_t alerm_buf[3];
    alerm_buf[0] = byte2BCD(data->minute);// MIN
    alerm_buf[1] = byte2BCD(data->hour);// HOUR
    alerm_buf[2] = byte2BCD(data->day);// DAY
    I2CWrite(RX8900_REG_MIN_ALARM, alerm_buf, 3);
}
/*
void SetAlerm() {
    Set_AIE();
    Set_WADA();
    uint8_t alerm_buf[3];
    alerm_buf[0] = byte2BCD(40);// MIN
    alerm_buf[1] = byte2BCD(20);// HOUR
    alerm_buf[2] = byte2BCD(26);// DAY
    I2CWrite(RX8900_REG_MIN_ALARM, alerm_buf, 3);
}
*/
void SetDefaultTime() {
    rtc_date_t rtcdate;
    rtcdate.year = 2001;
    rtcdate.month = 1;
    rtcdate.day = 1;
    rtcdate.hour = 0;
    rtcdate.minute = 0;
    rtcdate.second = 0;
    Rx8900_SetTime(&rtcdate);
}

void Rx8900_SetTime(rtc_date_t* data) {
    if (data == NULL) {
        return ;
    }
    Reset();
//    uint8_t dayOfWeek = subZeller(data->year, data->month, data->day);
    uint8_t time_buf[7];
    time_buf[0] = byte2BCD(data->second);
    time_buf[1] = byte2BCD(data->minute);
    time_buf[2] = byte2BCD(data->hour);
//    time_buf[3] = 1 << (dayOfWeek - 1);
    time_buf[4] = byte2BCD(data->day);
    time_buf[5] = byte2BCD(data->month);
    time_buf[6] = byte2BCD(data->year - 2000);
    I2CWrite(RX8900_REG_SEC, time_buf, 7);
}

void Rx8900_GetTime(rtc_date_t* data) {
    if (data == NULL) {
        return ;
    }
    uint8_t time_buf[7];
    I2CRead(RX8900_REG_SEC, time_buf, 7);
    data->second = BCD2Byte(time_buf[0]);
    data->minute = BCD2Byte(time_buf[1]);
    data->hour = BCD2Byte(time_buf[2]);
    data->day = BCD2Byte(time_buf[4]);
    data->month = BCD2Byte(time_buf[5]);
    data->year = BCD2Byte(time_buf[6]) + (2000);
}

esp_err_t Rx8900_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud) {
    vTaskDelay( pdMS_TO_TICKS(200) );
    rx8900_device = i2c_malloc_device(i2c_num, sda, scl, baud, RX8900_ADDR);
    if (rx8900_device == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    I2CWriteByte(RX8900_REG_EXTENSION_REGISTER, 0b00001000);
    I2CWriteByte(RX8900_REG_FLAG_REGSTER,       0b00000000);
    Reset();
    SetDefaultTime();
    Reset_AF();
//    SetAlerm();

    return ESP_OK;
}