// https://github.com/m5stack/M5Unit-ENV/blob/master/src/QMP6988.cpp
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "math.h"
#include "qmp6988.h"

#define QMP6988_ADDR (0x70)

#define QMP6988_U16_t unsigned short
#define QMP6988_S16_t short
#define QMP6988_U32_t unsigned int
#define QMP6988_S32_t int
#define QMP6988_U64_t unsigned long long
#define QMP6988_S64_t long long

#define QMP6988_CHIP_ID 0x5C

#define QMP6988_CHIP_ID_REG     0xD1
#define QMP6988_RESET_REG       0xE0 // Device reset register
#define QMP6988_DEVICE_STAT_REG 0xF3 // Device state register
#define QMP6988_CTRLMEAS_REG    0xF4 // Measurement Condition Control Register
// data
#define QMP6988_PRESSURE_MSB_REG    0xF7 // Pressure MSB Register
#define QMP6988_TEMPERATURE_MSB_REG 0xFA // Temperature MSB Reg

// compensation calculation
#define QMP6988_CALIBRATION_DATA_START (0xA0) // QMP6988 compensation coefficients
#define QMP6988_CALIBRATION_DATA_LENGTH 25

#define SHIFT_RIGHT_4_POSITION 4
#define SHIFT_LEFT_2_POSITION  2
#define SHIFT_LEFT_4_POSITION  4
#define SHIFT_LEFT_5_POSITION  5
#define SHIFT_LEFT_8_POSITION  8
#define SHIFT_LEFT_12_POSITION 12
#define SHIFT_LEFT_16_POSITION 16

// power mode
#define QMP6988_SLEEP_MODE  0x00
#define QMP6988_FORCED_MODE 0x01
#define QMP6988_NORMAL_MODE 0x03

#define QMP6988_CTRLMEAS_REG_MODE__POS 0
#define QMP6988_CTRLMEAS_REG_MODE__MSK 0x03
#define QMP6988_CTRLMEAS_REG_MODE__LEN 2

// oversampling
#define QMP6988_OVERSAMPLING_SKIPPED 0x00
#define QMP6988_OVERSAMPLING_1X      0x01
#define QMP6988_OVERSAMPLING_2X      0x02
#define QMP6988_OVERSAMPLING_4X      0x03
#define QMP6988_OVERSAMPLING_8X      0x04
#define QMP6988_OVERSAMPLING_16X     0x05
#define QMP6988_OVERSAMPLING_32X     0x06
#define QMP6988_OVERSAMPLING_64X     0x07

#define QMP6988_CTRLMEAS_REG_OSRST__POS 5
#define QMP6988_CTRLMEAS_REG_OSRST__MSK 0xE0
#define QMP6988_CTRLMEAS_REG_OSRST__LEN 3

#define QMP6988_CTRLMEAS_REG_OSRSP__POS 2
#define QMP6988_CTRLMEAS_REG_OSRSP__MSK 0x1C
#define QMP6988_CTRLMEAS_REG_OSRSP__LEN 3

// filter
#define QMP6988_FILTERCOEFF_OFF 0x00
#define QMP6988_FILTERCOEFF_2   0x01
#define QMP6988_FILTERCOEFF_4   0x02
#define QMP6988_FILTERCOEFF_8   0x03
#define QMP6988_FILTERCOEFF_16  0x04
#define QMP6988_FILTERCOEFF_32  0x05

#define QMP6988_CONFIG_REG             0xF1 //IIR filter co-efficient setting Register
#define QMP6988_CONFIG_REG_FILTER__POS 0
#define QMP6988_CONFIG_REG_FILTER__MSK 0x07
#define QMP6988_CONFIG_REG_FILTER__LEN 3

#define SUBTRACTOR 8388608

static const char *TAG = "QMP6988";

typedef struct _qmp6988_cali_data {
    QMP6988_S32_t COE_a0;
    QMP6988_S16_t COE_a1;
    QMP6988_S16_t COE_a2;
    QMP6988_S32_t COE_b00;
    QMP6988_S16_t COE_bt1;
    QMP6988_S16_t COE_bt2;
    QMP6988_S16_t COE_bp1;
    QMP6988_S16_t COE_b11;
    QMP6988_S16_t COE_bp2;
    QMP6988_S16_t COE_b12;
    QMP6988_S16_t COE_b21;
    QMP6988_S16_t COE_bp3;
} qmp6988_cali_data_t;

typedef struct _qmp6988_fk_data {
    float a0, b00;
    float a1, a2, bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_fk_data_t;

typedef struct _qmp6988_ik_data {
    QMP6988_S32_t a0, b00;
    QMP6988_S32_t a1, a2;
    QMP6988_S64_t bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_ik_data_t;

typedef struct _qmp6988_data {
    uint8_t slave;
    uint8_t chip_id;
    uint8_t power_mode;
    float temperature;
    float pressure;
    float altitude;
    qmp6988_cali_data_t qmp6988_cali;
    qmp6988_ik_data_t ik;
} qmp6988_data_t;

static I2CDevice_t qmp6988_device;
qmp6988_data_t qmp6988;

esp_err_t Qmp6988_CalibrationData() {
    esp_err_t status = ESP_OK;
    uint8_t a_data_uint8_tr[QMP6988_CALIBRATION_DATA_LENGTH] = {0};

    status = i2c_read_bytes(qmp6988_device, QMP6988_CALIBRATION_DATA_START, a_data_uint8_tr, QMP6988_CALIBRATION_DATA_LENGTH);
    if (status != ESP_OK) {
        ESP_LOGE(TAG, "qmp6988 read 0xA0 error!");
        return status;
    }

    qmp6988.qmp6988_cali.COE_a0 =
        (QMP6988_S32_t)(((a_data_uint8_tr[18] << SHIFT_LEFT_12_POSITION) |
                         (a_data_uint8_tr[19] << SHIFT_LEFT_4_POSITION) |
                         (a_data_uint8_tr[24] & 0x0f))
                        << 12);
    qmp6988.qmp6988_cali.COE_a0 = qmp6988.qmp6988_cali.COE_a0 >> 12;

    qmp6988.qmp6988_cali.COE_a1 =
        (QMP6988_S16_t)(((a_data_uint8_tr[20]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[21]);
    qmp6988.qmp6988_cali.COE_a2 =
        (QMP6988_S16_t)(((a_data_uint8_tr[22]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[23]);

    qmp6988.qmp6988_cali.COE_b00 =
        (QMP6988_S32_t)(((a_data_uint8_tr[0] << SHIFT_LEFT_12_POSITION) |
                         (a_data_uint8_tr[1] << SHIFT_LEFT_4_POSITION) |
                         ((a_data_uint8_tr[24] & 0xf0) >>
                          SHIFT_RIGHT_4_POSITION))
                        << 12);
    qmp6988.qmp6988_cali.COE_b00 = qmp6988.qmp6988_cali.COE_b00 >> 12;

    qmp6988.qmp6988_cali.COE_bt1 =
        (QMP6988_S16_t)(((a_data_uint8_tr[2]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[3]);
    qmp6988.qmp6988_cali.COE_bt2 =
        (QMP6988_S16_t)(((a_data_uint8_tr[4]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[5]);
    qmp6988.qmp6988_cali.COE_bp1 =
        (QMP6988_S16_t)(((a_data_uint8_tr[6]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[7]);
    qmp6988.qmp6988_cali.COE_b11 =
        (QMP6988_S16_t)(((a_data_uint8_tr[8]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[9]);
    qmp6988.qmp6988_cali.COE_bp2 =
        (QMP6988_S16_t)(((a_data_uint8_tr[10]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[11]);
    qmp6988.qmp6988_cali.COE_b12 =
        (QMP6988_S16_t)(((a_data_uint8_tr[12]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[13]);
    qmp6988.qmp6988_cali.COE_b21 =
        (QMP6988_S16_t)(((a_data_uint8_tr[14]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[15]);
    qmp6988.qmp6988_cali.COE_bp3 =
        (QMP6988_S16_t)(((a_data_uint8_tr[16]) << SHIFT_LEFT_8_POSITION) |
                        a_data_uint8_tr[17]);

    ESP_LOGI(TAG, "<-----------calibration data-------------->");
    ESP_LOGI(TAG, "COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]",
                qmp6988.qmp6988_cali.COE_a0, qmp6988.qmp6988_cali.COE_a1,
                qmp6988.qmp6988_cali.COE_a2, qmp6988.qmp6988_cali.COE_b00);
    ESP_LOGI(TAG, "COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]",
                qmp6988.qmp6988_cali.COE_bt1, qmp6988.qmp6988_cali.COE_bt2,
                qmp6988.qmp6988_cali.COE_bp1, qmp6988.qmp6988_cali.COE_b11);
    ESP_LOGI(TAG, "COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]",
                qmp6988.qmp6988_cali.COE_bp2, qmp6988.qmp6988_cali.COE_b12,
                qmp6988.qmp6988_cali.COE_b21, qmp6988.qmp6988_cali.COE_bp3);
    ESP_LOGI(TAG, "<-----------calibration data-------------->");

    qmp6988.ik.a0  = qmp6988.qmp6988_cali.COE_a0;   // 20Q4
    qmp6988.ik.b00 = qmp6988.qmp6988_cali.COE_b00;  // 20Q4

    qmp6988.ik.a1 = 3608L * (QMP6988_S32_t)qmp6988.qmp6988_cali.COE_a1 -
                    1731677965L;  // 31Q23
    qmp6988.ik.a2 = 16889L * (QMP6988_S32_t)qmp6988.qmp6988_cali.COE_a2 -
                    87619360L;  // 30Q47

    qmp6988.ik.bt1 = 2982L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bt1 +
                     107370906L;  // 28Q15
    qmp6988.ik.bt2 = 329854L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bt2 +
                     108083093L;  // 34Q38
    qmp6988.ik.bp1 = 19923L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bp1 +
                     1133836764L;  // 31Q20
    qmp6988.ik.b11 = 2406L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_b11 +
                     118215883L;  // 28Q34
    qmp6988.ik.bp2 = 3079L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bp2 -
                     181579595L;  // 29Q43
    qmp6988.ik.b12 = 6846L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_b12 +
                     85590281L;  // 29Q53
    qmp6988.ik.b21 = 13836L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_b21 +
                     79333336L;  // 29Q60
    qmp6988.ik.bp3 = 2915L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bp3 +
                     157155561L;  // 28Q65
    ESP_LOGI(TAG, "<----------- int calibration data -------------->");
    ESP_LOGI(TAG, "a0[%d]	a1[%d] a2[%d] b00[%d]", qmp6988.ik.a0,
                qmp6988.ik.a1, qmp6988.ik.a2, qmp6988.ik.b00);
    ESP_LOGI(TAG, "bt1[%lld]	bt2[%lld]	bp1[%lld]	b11[%lld]",
                qmp6988.ik.bt1, qmp6988.ik.bt2, qmp6988.ik.bp1, qmp6988.ik.b11);
    ESP_LOGI(TAG, "bp2[%lld]	b12[%lld]	b21[%lld]	bp3[%lld]",
                qmp6988.ik.bp2, qmp6988.ik.b12, qmp6988.ik.b21, qmp6988.ik.bp3);
    ESP_LOGI(TAG, "<----------- int calibration data -------------->");
    return ESP_OK;
}

QMP6988_S16_t Qmp6988_ConvTx02e(qmp6988_ik_data_t* ik, QMP6988_S32_t dt) {
    QMP6988_S16_t ret;
    QMP6988_S64_t wk1, wk2;

    // wk1: 60Q4 // bit size
    wk1 = ((QMP6988_S64_t)ik->a1 * (QMP6988_S64_t)dt);  // 31Q23+24-1=54 (54Q23)
    wk2 = ((QMP6988_S64_t)ik->a2 * (QMP6988_S64_t)dt) >>
          14;                                    // 30Q47+24-1=53 (39Q33)
    wk2 = (wk2 * (QMP6988_S64_t)dt) >> 10;       // 39Q33+24-1=62 (52Q23)
    wk2 = ((wk1 + wk2) / 32767) >> 19;           // 54,52->55Q23 (20Q04)
    ret = (QMP6988_S16_t)((ik->a0 + wk2) >> 4);  // 21Q4 -> 17Q0
    return ret;
}

QMP6988_S32_t Qmp6988_GetPressure02e(qmp6988_ik_data_t* ik, QMP6988_S32_t dp, QMP6988_S16_t tx) {
    QMP6988_S32_t ret;
    QMP6988_S64_t wk1, wk2, wk3;

    // wk1 = 48Q16 // bit size
    wk1 =
        ((QMP6988_S64_t)ik->bt1 * (QMP6988_S64_t)tx);  // 28Q15+16-1=43 (43Q15)
    wk2 = ((QMP6988_S64_t)ik->bp1 * (QMP6988_S64_t)dp) >>
          5;     // 31Q20+24-1=54 (49Q15)
    wk1 += wk2;  // 43,49->50Q15
    wk2 = ((QMP6988_S64_t)ik->bt2 * (QMP6988_S64_t)tx) >>
          1;                               // 34Q38+16-1=49 (48Q37)
    wk2 = (wk2 * (QMP6988_S64_t)tx) >> 8;  // 48Q37+16-1=63 (55Q29)
    wk3 = wk2;                             // 55Q29
    wk2 = ((QMP6988_S64_t)ik->b11 * (QMP6988_S64_t)tx) >>
          4;                               // 28Q34+16-1=43 (39Q30)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;  // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                            // 55,61->62Q29
    wk2 = ((QMP6988_S64_t)ik->bp2 * (QMP6988_S64_t)dp) >>
          13;                              // 29Q43+24-1=52 (39Q30)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;  // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                            // 62,61->63Q29
    wk1 += wk3 >> 14;                      // Q29 >> 14 -> Q15
    wk2 =
        ((QMP6988_S64_t)ik->b12 * (QMP6988_S64_t)tx);  // 29Q53+16-1=45 (45Q53)
    wk2 = (wk2 * (QMP6988_S64_t)tx) >> 22;             // 45Q53+16-1=61 (39Q31)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;              // 39Q31+24-1=62 (61Q30)
    wk3 = wk2;                                         // 61Q30
    wk2 = ((QMP6988_S64_t)ik->b21 * (QMP6988_S64_t)tx) >>
          6;                                // 29Q60+16-1=45 (39Q54)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 23;  // 39Q54+24-1=62 (39Q31)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;   // 39Q31+24-1=62 (61Q20)
    wk3 += wk2;                             // 61,61->62Q30
    wk2 = ((QMP6988_S64_t)ik->bp3 * (QMP6988_S64_t)dp) >>
          12;                               // 28Q65+24-1=51 (39Q53)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 23;  // 39Q53+24-1=62 (39Q30)
    wk2 = (wk2 * (QMP6988_S64_t)dp);        // 39Q30+24-1=62 (62Q30)
    wk3 += wk2;                             // 62,62->63Q30
    wk1 += wk3 >> 15;                       // Q30 >> 15 = Q15
    wk1 /= 32767L;
    wk1 >>= 11;      // Q15 >> 7 = Q4
    wk1 += ik->b00;  // Q4 + 20Q4
    // wk1 >>= 4; // 28Q4 -> 24Q0
    ret = (QMP6988_S32_t)wk1;
    return ret;
}

esp_err_t Qmp6988_SoftwareReset() {
    esp_err_t ret = ESP_OK;
    ret = i2c_write_byte(qmp6988_device, QMP6988_RESET_REG, 0xe6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "softwareReset fail!!!");
        return ret;
    }
    vTaskDelay( pdMS_TO_TICKS(20) );
    ret = i2c_write_byte(qmp6988_device, QMP6988_RESET_REG, 0x00);
    return ESP_OK;
}

void Qmp6988_SetpPowermode(int power_mode) {
    uint8_t data;

    ESP_LOGI(TAG, "qmp_set_powermode %d", power_mode);

    qmp6988.power_mode = power_mode;
    i2c_read_byte(qmp6988_device, QMP6988_CTRLMEAS_REG, &data);
    data = data & 0xfc;
    if (power_mode == QMP6988_SLEEP_MODE) {
        data |= 0x00;
    } else if (power_mode == QMP6988_FORCED_MODE) {
        data |= 0x01;
    } else if (power_mode == QMP6988_NORMAL_MODE) {
        data |= 0x03;
    }
    i2c_write_byte(qmp6988_device, QMP6988_CTRLMEAS_REG, data);

    ESP_LOGI(TAG, "qmp_set_powermode 0xf4=0x%x", data);

    vTaskDelay( pdMS_TO_TICKS(20) );
}

void Qmp6988_SetFilter(uint8_t filter) {
    uint8_t data;

    data = (filter & 0x03);
    i2c_write_byte(qmp6988_device, QMP6988_CONFIG_REG, data);

    vTaskDelay( pdMS_TO_TICKS(20) );
}

void Qmp6988_SetOversamplingP(uint8_t oversampling_p) {
    uint8_t data;

    i2c_read_byte(qmp6988_device, QMP6988_CTRLMEAS_REG, &data);
    data &= 0xe3;
    data |= (oversampling_p << 2);
    i2c_write_byte(qmp6988_device, QMP6988_CTRLMEAS_REG, data);

    vTaskDelay( pdMS_TO_TICKS(20) );
}

void Qmp6988_SetOversamplingT(uint8_t oversampling_t) {
    uint8_t data;

    i2c_read_byte(qmp6988_device, QMP6988_CTRLMEAS_REG, &data);
    data &= 0x1f;
    data |= (oversampling_t << 5);
    i2c_write_byte(qmp6988_device, QMP6988_CTRLMEAS_REG, data);

    vTaskDelay( pdMS_TO_TICKS(20) );
}

float Qmp6988_CalcAltitude(float pressure, float temp) {
    float altitude;

    altitude =
        (pow((101325 / pressure), 1 / 5.257) - 1) * (temp + 273.15) / 0.0065;
    ESP_LOGI(TAG, "altitude = %f", altitude);
    return altitude;
}

float Qmp6988_CalcPressure() {
    esp_err_t err = ESP_OK;
    QMP6988_U32_t P_read, T_read;
    QMP6988_S32_t P_raw, T_raw;
    uint8_t a_data_uint8_tr[6] = {0};
    QMP6988_S32_t T_int, P_int;

    // press
    err = i2c_read_bytes(qmp6988_device, QMP6988_PRESSURE_MSB_REG, a_data_uint8_tr, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "qmp6988 read press raw error! ");
        return 0.0f;
    }
    P_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[0]))
                              << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[1]))
                              << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[2]));
    P_raw  = (QMP6988_S32_t)(P_read - SUBTRACTOR);

    T_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[3]))
                              << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[4]))
                              << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[5]));
    T_raw  = (QMP6988_S32_t)(T_read - SUBTRACTOR);

    T_int               = Qmp6988_ConvTx02e(&(qmp6988.ik), T_raw);
    P_int               = Qmp6988_GetPressure02e(&(qmp6988.ik), P_raw, T_int);
    qmp6988.temperature = (float)T_int / 256.0f;
    qmp6988.pressure    = (float)P_int / 16.0f;

    return qmp6988.pressure;
}

float Qmp6988_CalcTemperature() {
    esp_err_t err = ESP_OK;
    QMP6988_U32_t P_read, T_read;
    QMP6988_S32_t P_raw, T_raw;
    uint8_t a_data_uint8_tr[6] = {0};
    QMP6988_S32_t T_int, P_int;

    // press
    err = i2c_read_bytes(qmp6988_device, QMP6988_PRESSURE_MSB_REG, a_data_uint8_tr, 6);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "qmp6988 read press raw error! ");
        return 0.0f;
    }
    P_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[0]))
                              << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[1]))
                              << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[2]));
    P_raw  = (QMP6988_S32_t)(P_read - SUBTRACTOR);

    // temp
    err = i2c_read_bytes(qmp6988_device, QMP6988_TEMPERATURE_MSB_REG, a_data_uint8_tr, 3);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "qmp6988 read temp raw error! ");
    }
    T_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[3]))
                              << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[4]))
                              << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[5]));
    T_raw  = (QMP6988_S32_t)(T_read - SUBTRACTOR);

    T_int               = Qmp6988_ConvTx02e(&(qmp6988.ik), T_raw);
    P_int               = Qmp6988_GetPressure02e(&(qmp6988.ik), P_raw, T_int);
    qmp6988.temperature = (float)T_int / 256.0f;
    qmp6988.pressure    = (float)P_int / 16.0f;

    return qmp6988.temperature;
}

float Qmp6988_GetPressure() {
    return Qmp6988_CalcPressure();
}

float Qmp6988_GetTemperature() {
    return Qmp6988_CalcTemperature();
}

esp_err_t Qmp6988_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t baud) {
    qmp6988_device = i2c_malloc_device(i2c_num, sda, scl, baud, QMP6988_ADDR);
    if (qmp6988_device == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    Qmp6988_SoftwareReset();
    Qmp6988_CalibrationData();
    Qmp6988_SetpPowermode(QMP6988_NORMAL_MODE);
    Qmp6988_SetFilter(QMP6988_FILTERCOEFF_4);
    Qmp6988_SetOversamplingP(QMP6988_OVERSAMPLING_8X);
    Qmp6988_SetOversamplingT(QMP6988_OVERSAMPLING_1X);

    return ESP_OK;
}
