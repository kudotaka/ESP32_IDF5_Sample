#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"

#if CONFIG_SOFTWARE_ESP_ULP_TEST
#include <stdio.h>
#include "esp_sleep.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "ulp_riscv.h"
#include "ulp_main.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_ulp_program(void);
#endif

#if CONFIG_SOFTWARE_UNIT_RTC_SUPPORT
#include "esp_sntp.h"
#endif

#if ( CONFIG_HARDWARE_MODEL_EPS32S3 \
   || CONFIG_HARDWARE_MODEL_XIAO_EPS32C3 \
   || CONFIG_SOFTWARE_ESP_LCD_UI_SUPPORT \
   || CONFIG_SOFTWARE_MODEL_SSD1306_I2C \
   || CONFIG_SOFTWARE_ESP_WIFI_SUPPORT \
   || CONFIG_SOFTWARE_UNIT_ENV2_SUPPORT \
   || CONFIG_SOFTWARE_UNIT_ENV3_SUPPORT \
   || CONFIG_SOFTWARE_UNIT_SK6812_SUPPORT \
   || CONFIG_SOFTWARE_UNIT_4DIGIT_DISPLAY_SUPPORT \
   || CONFIG_SOFTWARE_UNIT_6DIGIT_DISPLAY_SUPPORT \
   || CONFIG_SOFTWARE_UNIT_BUTTON_SUPPORT \
   || CONFIG_SOFTWARE_UNIT_LED_SUPPORT \
   || CONFIG_SOFTWARE_UNIT_RTC_SUPPORT \
   || CONFIG_SOFTWARE_I2C_LCD_ST7032_SUPPORT )
//#include "esp_unit.h"
#include "esp_moduleunit.h"
#endif

#if CONFIG_SOFTWARE_UNIT_SK6812_SUPPORT
#ifdef BUILD_ESP_IDF_VERSION_5
#include <string.h>
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_LED_NUMBERS_1       3
#define RMT_LED_STRIP_LED_NUMBERS_2       8
#define RMT_LED_STRIP_CHASE_SPEED_MS      100
static uint8_t led_strip_pixels_1[RMT_LED_STRIP_LED_NUMBERS_1 * 3];
static uint8_t led_strip_pixels_2[RMT_LED_STRIP_LED_NUMBERS_2 * 3];
#endif
#endif


static const char *TAG = "MY-MAIN";


#ifdef CONFIG_SOFTWARE_BUZZER_SUPPORT
TaskHandle_t xBuzzer;
static void vLoopBuzzerTask(void* pvParameters) {
    ESP_LOGI(TAG, "Buzzer is start!");

    Buzzer_Init(PORT_D3_PIN);
    uint32_t DUTY_MAX_VALUE = 8190;
    uint32_t duty[] = {DUTY_MAX_VALUE/4*2};
    while (1) {
        vTaskSuspend(NULL);
        for (uint32_t i = 0; i < sizeof(duty)/sizeof(uint32_t); i++)
        {
            ESP_LOGI(TAG, "Buzzer duty:%ld", duty[i]);
            Buzzer_Play_Duty_Frequency(duty[i], 261.6);
            vTaskDelay( pdMS_TO_TICKS(500) );
            Buzzer_Play_Duty_Frequency(duty[i], 293.665);
            vTaskDelay( pdMS_TO_TICKS(500) );
            Buzzer_Play_Duty_Frequency(duty[i], 329.63);
            vTaskDelay( pdMS_TO_TICKS(500) );
            Buzzer_Play_Duty_Frequency(duty[i], 349.228);
            vTaskDelay( pdMS_TO_TICKS(500) );
            Buzzer_Play_Duty_Frequency(duty[i], 391.995);
            vTaskDelay( pdMS_TO_TICKS(500) );
            Buzzer_Play_Duty_Frequency(duty[i], 440);
            vTaskDelay( pdMS_TO_TICKS(500) );
            Buzzer_Play_Duty_Frequency(duty[i], 493.883);
            vTaskDelay( pdMS_TO_TICKS(500) );
            Buzzer_Play_Duty_Frequency(duty[i], 523.251);
            vTaskDelay( pdMS_TO_TICKS(500) );

            Buzzer_Stop();
            vTaskDelay( pdMS_TO_TICKS(5000) );
        }
    }

    vTaskDelete(NULL); // Should never get to here...
}
#endif

#ifdef CONFIG_SOFTWARE_UNIT_BUTTON_SUPPORT
TaskHandle_t xButton;
Button_t* button1 = NULL;

static void button_task(void* pvParameters) {
    ESP_LOGI(TAG, "start button_task");
    Button_Init();

#if CONFIG_HARDWARE_MODEL_EPS32S3
    if (Button_Enable(PORT_D4_PIN, BUTTON_ACTIVE_LOW) == ESP_OK) {
        button1 = Button_Attach(PORT_D4_PIN, BUTTON_ACTIVE_LOW);
        if (button1 == NULL) {
            ESP_LOGE(TAG, "Button_Attach : button1");
        }
    }
#elif CONFIG_HARDWARE_MODEL_XIAO_EPS32C3
    if (Button_Enable(PORT_D9_PIN, BUTTON_ACTIVE_LOW) == ESP_OK) {
        button1 = Button_Attach(PORT_D9_PIN, BUTTON_ACTIVE_LOW);
        if (button1 == NULL) {
            ESP_LOGE(TAG, "Button_Attach : button1");
        }
    }
#endif

    while(1){
        if (button1 != NULL) {
            if (Button_WasPressed(button1)) {
                ESP_LOGI(TAG, "button1 BUTTON PRESSED!");
#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
                ui_button_label_update(true);
#endif
            }
            if (Button_WasReleased(button1)) {
                ESP_LOGI(TAG, "button1 BUTTON RELEASED!");
#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
                ui_button_label_update(false);
#endif
            }
            if (Button_WasLongPress(button1, pdMS_TO_TICKS(1000))) { // 1Sec
                ESP_LOGI(TAG, "button1 BUTTON LONGPRESS!");
#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
                ui_button_label_update(false);
#endif
#ifdef CONFIG_SOFTWARE_BUZZER_SUPPORT
            vTaskResume(xBuzzer);
#endif
            }
        }

        vTaskDelay(pdMS_TO_TICKS(80));
    }
    vTaskDelete(NULL); // Should never get to here...
}
#endif

#if CONFIG_SOFTWARE_UNIT_LED_SUPPORT
// SELECT GPIO_NUM_XX OR PORT_DX_PIN
TaskHandle_t xExternalLED;
Led_t* led_ext1;
static void external_led_task(void* pvParameters) {
    ESP_LOGI(TAG, "start external_led_task");
    Led_Init();

#if CONFIG_HARDWARE_MODEL_EPS32S3
    if (Led_Enable(PORT_D5_PIN, LED_ACTIVE_HIGH) == ESP_OK) {
        led_ext1 = Led_Attach(PORT_D5_PIN, LED_ACTIVE_HIGH);
    }
#elif CONFIG_HARDWARE_MODEL_XIAO_EPS32C3
    if (Led_Enable(PORT_D10_PIN, LED_ACTIVE_HIGH) == ESP_OK) {
        led_ext1 = Led_Attach(PORT_D10_PIN, LED_ACTIVE_HIGH);
    }
#endif

    while(1){
        Led_OnOff(led_ext1, true);
        vTaskDelay(pdMS_TO_TICKS(300));
        Led_OnOff(led_ext1, false);
        vTaskDelay(pdMS_TO_TICKS(700));
    }
    vTaskDelete(NULL); // Should never get to here...
}
#endif

#ifdef CONFIG_SOFTWARE_UNIT_RTC_SUPPORT
TaskHandle_t xRtc;
TaskHandle_t xClock;
static bool g_timeInitialized = false;
static bool g_firstTimeInitialized = false;
const char servername[] = "ntp.jst.mfeed.ad.jp";

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
    g_timeInitialized = true;
}

void vLoopRtcTask(void *pvParametes)
{
    //RTC
    ESP_LOGI(TAG, "start vLoopRtcTask. TaskDelayTime");

    g_firstTimeInitialized = false;

    // Set timezone to Japan Standard Time
    setenv("TZ", "JST-9", 1);
    tzset();

    ESP_LOGI(TAG, "ServerName:%s", servername);
    sntp_setservername(0, servername);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    while (1) {
        if (wifi_isConnected() == ESP_OK) {
            sntp_init();
        } else {
            vTaskDelay( pdMS_TO_TICKS(60000) );
            continue;
        }

        ESP_LOGI(TAG, "Waiting for time synchronization with SNTP server");
        while (!g_timeInitialized)
        {
            vTaskDelay( pdMS_TO_TICKS(5000) );
        }

        time_t now = 0;
        struct tm timeinfo = {0};
        time(&now);
        localtime_r(&now, &timeinfo);
        char str1[72] = {0};
        sprintf(str1,"NTP Update : %04d/%02d/%02d %02d:%02d", timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min);
        ESP_LOGI(TAG, "%s", str1);

        rtc_date_t rtcdate;
        rtcdate.year = timeinfo.tm_year+1900;
        rtcdate.month = timeinfo.tm_mon+1;
        rtcdate.day = timeinfo.tm_mday;
        rtcdate.hour = timeinfo.tm_hour;
        rtcdate.minute = timeinfo.tm_min;
        rtcdate.second = timeinfo.tm_sec;
#if CONFIG_SOFTWARE_MODEL_RTC_PCF8563
        PCF8563_SetTime(&rtcdate);
#elif CONFIG_SOFTWARE_MODEL_RTC_RX8900
        Rx8900_SetTime(&rtcdate);
#else
#endif

        g_timeInitialized = false;
        g_firstTimeInitialized = true;
        sntp_stop();

        vTaskDelay( pdMS_TO_TICKS(600000) );
    }
}

void vLoopClockTask(void *pvParametes)
{
    //PCF8563
    ESP_LOGI(TAG, "start vLoopClockTask.");

    esp_err_t ret = ESP_OK;
#if CONFIG_SOFTWARE_MODEL_RTC_PCF8563
    ret = PCF8563_Init(I2C_NUM_0, PORT_SDA_PIN, PORT_SCL_PIN, PORT_I2C_STANDARD_BAUD);
#elif CONFIG_SOFTWARE_MODEL_RTC_RX8900
    ret = Rx8900_Init(I2C_NUM_0, PORT_SDA_PIN, PORT_SCL_PIN, PORT_I2C_STANDARD_BAUD);
#else
#endif
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTC Init Error");
        return;
    }

    // Set timezone to Japan Standard Time
    setenv("TZ", "JST-9", 1);
    tzset();

    while (1) {
        rtc_date_t rtcdate;
#if CONFIG_SOFTWARE_MODEL_RTC_PCF8563
        PCF8563_GetTime(&rtcdate);
#elif CONFIG_SOFTWARE_MODEL_RTC_RX8900
        while (!g_firstTimeInitialized)
        {
            vTaskDelay( pdMS_TO_TICKS(5000) );
        }
        Rx8900_GetTime(&rtcdate);
#else
#endif
        char str1[30] = {0};
        sprintf(str1,"%04d/%02d/%02d %02d:%02d:%02d", rtcdate.year, rtcdate.month, rtcdate.day, rtcdate.hour, rtcdate.minute, rtcdate.second);
#if CONFIG_SOFTWARE_MODEL_SSD1306_I2C
        ui_datetime_set(str1);
#else
        // debug
        ESP_LOGI(TAG, "%s", str1);
#endif
        vTaskDelay( pdMS_TO_TICKS(990) );
    }
}
#endif

#if CONFIG_SOFTWARE_UNIT_SK6812_SUPPORT
TaskHandle_t xExternalRGBLedBlink;
#ifdef BUILD_ESP_IDF_VERSION_5
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}
#else
#endif

void vexternal_LoopRGBLedBlinkTask(void *pvParametes)
{
    ESP_LOGI(TAG, "start vexternal_LoopRGBLedBlinkTask");
#ifdef BUILD_ESP_IDF_VERSION_5

    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan_1 = NULL;
    rmt_channel_handle_t led_chan_2 = NULL;
    rmt_tx_channel_config_t tx_chan_config_1 = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = PORT_D7_PIN,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_1, &led_chan_1));
    rmt_tx_channel_config_t tx_chan_config_2 = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = PORT_D6_PIN,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_2, &led_chan_2));

    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan_1));
    ESP_ERROR_CHECK(rmt_enable(led_chan_2));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    uint8_t brightness = 10;
    float b = brightness / 255.0;
    ESP_LOGI(TAG, "brightness %4.2f", b);
/*
    uint8_t blink_count = 3;
    uint8_t colors[][3] = {
        {  0,  0,255},//SK6812_COLOR_BLUE
        {  0,255,  0},//SK6812_COLOR_LIME
        {  0,255,255},//SK6812_COLOR_AQUA
        {255,  0,  0},//SK6812_COLOR_RED
        {255,  0,255},//SK6812_COLOR_MAGENTA
        {255,255,  0},//SK6812_COLOR_YELLOW
        {255,255,255}//K6812_COLOR_WHITE
    };
    while (1) {
        for (uint8_t c = 0; c < sizeof(colors)/sizeof(uint8_t)/3; c++) {
            for (uint8_t i = 0; i < blink_count; i++) {
                for (int j = 0; j < RMT_LED_STRIP_LED_NUMBERS_1; j++) {
                    // SK6812 => GRB
                    led_strip_pixels_1[j * 3 + 0] = b * colors[c][1];//green
                    led_strip_pixels_1[j * 3 + 1] = b * colors[c][0];//red
                    led_strip_pixels_1[j * 3 + 2] = b * colors[c][2];//blue
                }
                ESP_ERROR_CHECK(rmt_transmit(led_chan_1, led_encoder, led_strip_pixels_1, sizeof(led_strip_pixels_1), &tx_config));

                for (int j = 0; j < RMT_LED_STRIP_LED_NUMBERS_2; j++) {
                    uint8_t pos = 6 - c;
                    if (pos < 0) {
                        pos = 0;
                    }
                    // SK6812 => GRB
                    led_strip_pixels_2[j * 3 + 0] = b * colors[pos][1];//green
                    led_strip_pixels_2[j * 3 + 1] = b * colors[pos][0];//red
                    led_strip_pixels_2[j * 3 + 2] = b * colors[pos][2];//blue
                }
                ESP_ERROR_CHECK(rmt_transmit(led_chan_2, led_encoder, led_strip_pixels_2, sizeof(led_strip_pixels_2), &tx_config));
                vTaskDelay(pdMS_TO_TICKS(1000));

                memset(led_strip_pixels_1, 0, sizeof(led_strip_pixels_1));
                memset(led_strip_pixels_2, 0, sizeof(led_strip_pixels_2));
                ESP_ERROR_CHECK(rmt_transmit(led_chan_1, led_encoder, led_strip_pixels_1, sizeof(led_strip_pixels_1), &tx_config));
                ESP_ERROR_CHECK(rmt_transmit(led_chan_2, led_encoder, led_strip_pixels_2, sizeof(led_strip_pixels_2), &tx_config));
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
*/

    while (1) {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < RMT_LED_STRIP_LED_NUMBERS_1; j += 3) {
                // Build RGB pixels
                hue = j * 360 / RMT_LED_STRIP_LED_NUMBERS_1 + start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                led_strip_pixels_1[j * 3 + 0] = b * green;
                led_strip_pixels_1[j * 3 + 1] = b * blue;
                led_strip_pixels_1[j * 3 + 2] = b * red;
            }
            for (int j = i; j < RMT_LED_STRIP_LED_NUMBERS_2; j += 3) {
                // Build RGB pixels
                hue = j * 360 / RMT_LED_STRIP_LED_NUMBERS_2 + start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                led_strip_pixels_2[j * 3 + 0] = b * green;
                led_strip_pixels_2[j * 3 + 1] = b * blue;
                led_strip_pixels_2[j * 3 + 2] = b * red;
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(rmt_transmit(led_chan_1, led_encoder, led_strip_pixels_1, sizeof(led_strip_pixels_1), &tx_config));
            ESP_ERROR_CHECK(rmt_transmit(led_chan_2, led_encoder, led_strip_pixels_2, sizeof(led_strip_pixels_2), &tx_config));
            vTaskDelay(pdMS_TO_TICKS(RMT_LED_STRIP_CHASE_SPEED_MS));
            memset(led_strip_pixels_1, 0, sizeof(led_strip_pixels_1));
            memset(led_strip_pixels_2, 0, sizeof(led_strip_pixels_2));
            ESP_ERROR_CHECK(rmt_transmit(led_chan_1, led_encoder, led_strip_pixels_1, sizeof(led_strip_pixels_1), &tx_config));
            ESP_ERROR_CHECK(rmt_transmit(led_chan_2, led_encoder, led_strip_pixels_2, sizeof(led_strip_pixels_2), &tx_config));
            vTaskDelay(pdMS_TO_TICKS(RMT_LED_STRIP_CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
#else
    pixel_settings_t px_ext1;
    uint8_t blink_count = 5;
    uint32_t colors[] = {SK6812_COLOR_BLUE, SK6812_COLOR_LIME, SK6812_COLOR_AQUA
                    , SK6812_COLOR_RED, SK6812_COLOR_MAGENTA, SK6812_COLOR_YELLOW
                    , SK6812_COLOR_WHITE};
    Sk6812_Init(&px_ext1, PORT_D5_PIN, RMT_CHANNEL_0, 3);
    while (1) {
        for (uint8_t c = 0; c < sizeof(colors)/sizeof(uint32_t); c++) {
            for (uint8_t i = 0; i < blink_count; i++) {
                Sk6812_SetAllColor(&px_ext1, colors[c]);
                Sk6812_Show(&px_ext1, RMT_CHANNEL_0);
                vTaskDelay(pdMS_TO_TICKS(1000));

                Sk6812_SetAllColor(&px_ext1, SK6812_COLOR_OFF);
                Sk6812_Show(&px_ext1, RMT_CHANNEL_0);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
#endif
}
#endif

#ifdef CONFIG_SOFTWARE_UNIT_ENV2_SUPPORT
TaskHandle_t xUnitEnv2;
void vLoopUnitEnv2Task(void *pvParametes)
{
    ESP_LOGI(TAG, "start I2C Sht3x");
    esp_err_t ret = ESP_OK;
    ret = Sht3x_Init(I2C_NUM_0, PORT_SDA_PIN, PORT_SCL_PIN, PORT_I2C_STANDARD_BAUD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sht3x_Init Error");
        return;
    }
    ESP_LOGI(TAG, "start I2C Bmp280");
    ret = Bmp280_Init(I2C_NUM_0, PORT_SDA_PIN, PORT_SCL_PIN, PORT_I2C_STANDARD_BAUD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bmp280_Init Error");
        return;
    }

    while (1) {
        ret = Sht3x_Read();
        if (ret == ESP_OK) {
            vTaskDelay( pdMS_TO_TICKS(100) );
            ESP_LOGI(TAG, "temperature:%f, humidity:%f, pressure:%fhPa", Sht3x_GetTemperature(), Sht3x_GetHumidity(), Bmp280_GetPressure()/100 );
#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
            ui_temperature_update( Sht3x_GetIntTemperature() );
            ui_humidity_update( Sht3x_GetIntHumidity() );
#endif
        } else {
            ESP_LOGE(TAG, "Sht3x_Read() is error code:%d", ret);
            vTaskDelay( pdMS_TO_TICKS(10000) );
        }

        vTaskDelay( pdMS_TO_TICKS(5000) );
    }
}
#endif

#ifdef CONFIG_SOFTWARE_UNIT_ENV3_SUPPORT
TaskHandle_t xUnitEnv3;
void vLoopUnitEnv3Task(void *pvParametes)
{
    ESP_LOGI(TAG, "start I2C Sht3x");
    esp_err_t ret = ESP_OK;
    ret = Sht3x_Init(I2C_NUM_0, PORT_SDA_PIN, PORT_SCL_PIN, PORT_I2C_STANDARD_BAUD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sht3x_Init Error");
        return;
    }
    ESP_LOGI(TAG, "start I2C Qmp6988");
    ret = Qmp6988_Init(I2C_NUM_0, PORT_SDA_PIN, PORT_SCL_PIN, PORT_I2C_STANDARD_BAUD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Qmp6988_Init Error");
        return;
    }

    while (1) {
        ret = Sht3x_Read();
        if (ret == ESP_OK) {
            vTaskDelay( pdMS_TO_TICKS(100) );
            ESP_LOGI(TAG, "temperature:%f, humidity:%f, pressure:%fhPa", Sht3x_GetTemperature(), Sht3x_GetHumidity(), Qmp6988_GetPressure()/100 );
#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
            ui_temperature_update( Sht3x_GetIntTemperature() );
            ui_humidity_update( Sht3x_GetIntHumidity() );
#endif
        } else {
            ESP_LOGE(TAG, "Sht3x_Read() is error code:%d", ret);
            vTaskDelay( pdMS_TO_TICKS(10000) );
        }

        vTaskDelay( pdMS_TO_TICKS(5000) );
    }
}
#endif

#ifdef CONFIG_SOFTWARE_I2C_LCD_ST7032_SUPPORT
TaskHandle_t xLcdSt7032;
uint8_t degree[] = {
    0b11000,
    0b11000,
    0b00110,
    0b01001,
    0b01000,
    0b01001,
    0b00110,
    0b00000,
};

void vLoopLcdSt7032Task(void *pvParametes)
{
    ESP_LOGI(TAG, "start LCD ST7032");

    esp_err_t ret = ESP_OK;
#ifdef CONFIG_SOFTWARE_USE_BACKLIGHT
    ret = St7032_Init(I2C_NUM_0, PORT_SDA_PIN, PORT_SCL_PIN, PORT_I2C_STANDARD_BAUD, PORT_D45_PIN);
#else
    ret = St7032_Init(I2C_NUM_0, PORT_SDA_PIN, PORT_SCL_PIN, PORT_I2C_STANDARD_BAUD);
#endif
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD ST7032 INIT Error");
        return;
    }
    ESP_LOGI(TAG, "LCD ST7032 INIT is OK!");

// Sample ALL Character
#ifdef CONFIG_SOFTWARE_USE_BACKLIGHT
        St7032_Backlight_OnOff(PORT_LEVEL_HIGH);
#endif
    while(1) {
        St7032_ClearDisplay();

        for(uint8_t i = 0; i < 0xFF; i++) {
            St7032_CursorByPotision(1, 1);
            for(uint8_t j = 0; j < 8; j++) {
                St7032_WriteData(i);
            }
            St7032_ShowDisplayByPotision(2, 1, 0x30);
            St7032_ShowDisplayByPotision(2, 2, 0x78);
            St7032_ShowDisplayByPotision(2, 3, St7032_ConvertHexNumber((i & 0xF0) >> 4));
            St7032_ShowDisplayByPotision(2, 4, St7032_ConvertHexNumber((i & 0x0F)));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

/*
// Sample
    St7032_CreateOrignalCharacter(0x01, degree, sizeof(degree)/sizeof(uint8_t));

    uint8_t moji1[] = "Hello";
    uint8_t moji2[] = "  World!";
    while(1) {
        St7032_ClearDisplay();
        St7032_ReturnHome();
#ifdef CONFIG_SOFTWARE_USE_BACKLIGHT
        St7032_Backlight_OnOff(PORT_LEVEL_HIGH);
#endif
        for(uint8_t i = 0; i < sizeof(moji1)/sizeof(uint8_t); i++) {
            St7032_WriteData(moji1[i]);
        }
       
        vTaskDelay(pdMS_TO_TICKS(1000));
        St7032_CursorByPotision(2, 1);
        for(uint8_t i = 0; i < sizeof(moji2)/sizeof(uint8_t); i++) {
            St7032_WriteData(moji2[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        St7032_ClearDisplay();
        vTaskDelay(pdMS_TO_TICKS(1000));
#ifdef CONFIG_SOFTWARE_USE_BACKLIGHT
        St7032_Backlight_OnOff(PORT_LEVEL_LOW);
#endif
        St7032_ShowDisplayByPotision(1, 2, 0x32);
        St7032_ShowDisplayByPotision(1, 3, 0x35);
        St7032_ShowDisplayByPotision(1, 4, 0x2E);
        St7032_ShowDisplayByPotision(1, 5, 0x39);
        St7032_ShowDisplayByPotision(1, 7, 0x01);
        St7032_ShowDisplayByPotision(2, 2, 0x34);
        St7032_ShowDisplayByPotision(2, 3, 0x30);
        St7032_ShowDisplayByPotision(2, 4, 0x2E);
        St7032_ShowDisplayByPotision(2, 5, 0x35);
        St7032_ShowDisplayByPotision(2, 7, 0x25);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
*/
    vTaskDelete(NULL); // Should never get to here...
}
#endif

#if ( CONFIG_SOFTWARE_UNIT_4DIGIT_DISPLAY_SUPPORT || CONFIG_SOFTWARE_UNIT_6DIGIT_DISPLAY_SUPPORT )
TaskHandle_t xDigitDisplay;
DigitDisplay_t* digitdisplay_1;
void vLoopUnitDigitDisplayTask(void *pvParametes)
{
    ESP_LOGI(TAG, "start Digit Display");

// Sample Rowdata
//  --0x01--
// |        |
//0x20     0x02
// |        |
//  --0x40- -
// |        |
//0x10     0x04
// |        |
//  --0x08--   0x80


// Sample time
#ifdef CONFIG_SOFTWARE_UNIT_RTC_SUPPORT
    rtc_date_t rtcdate;
    uint8_t currentHour = 0;
    uint8_t currentMinute = 0;
    uint8_t currentSecond = 0;

    Tm1637_Init();
    if (Tm1637_Enable(PORT_D6_PIN, PORT_D7_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(PORT_D6_PIN, PORT_D7_PIN, BRIGHT_DARKEST);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
        vTaskDelete(NULL);
    }
    Tm1637_ClearDisplay(digitdisplay_1);
    vTaskDelay(pdMS_TO_TICKS(100));
    while(1) {
#if CONFIG_SOFTWARE_MODEL_RTC_PCF8563
        PCF8563_GetTime(&rtcdate);
#elif CONFIG_SOFTWARE_MODEL_RTC_RX8900
        Rx8900_GetTime(&rtcdate);
#else
#endif
        if (currentHour != rtcdate.hour) {
            currentHour = rtcdate.hour;
            Tm1637_DisplayBitAddPoint(digitdisplay_1, 0, currentHour / 10, POINT_OFF);
            Tm1637_DisplayBitAddPoint(digitdisplay_1, 1, currentHour % 10, POINT_ON);
        }
        if (currentMinute != rtcdate.minute) {
            currentMinute = rtcdate.minute;
            Tm1637_DisplayBitAddPoint(digitdisplay_1, 2, currentMinute / 10, POINT_OFF);
            Tm1637_DisplayBitAddPoint(digitdisplay_1, 3, currentMinute % 10, POINT_ON);
        }
        if (currentSecond != rtcdate.second) {
            currentSecond = rtcdate.second;
            Tm1637_DisplayBitAddPoint(digitdisplay_1, 4, currentSecond / 10, POINT_OFF);
            Tm1637_DisplayBitAddPoint(digitdisplay_1, 5, currentSecond % 10, POINT_OFF);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL); // Should never get to here...
#endif

/*
// Sample animetion
    uint8_t anime[] = {0x00, 0x30, 0x38, 0x78, 0x79, 0x7f};
    uint8_t animeCurrent = 0;
    uint8_t animeMax = sizeof(anime)/sizeof(uint8_t);
    uint8_t animeDigitPosition = 1;
    uint8_t animeDigitMax = DIGIT_COUNT;
    uint8_t data = 0x00;
    Tm1637_Init();
    if (Tm1637_Enable(PORT_D9_PIN, PORT_D10_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(PORT_D9_PIN, PORT_D10_PIN, BRIGHT_TYPICAL);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
        vTaskDelete(NULL);
    }
    while(1) {
        Tm1637_ClearDisplay(digitdisplay_1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        for (animeDigitPosition = 1; animeDigitPosition < animeDigitMax+1; animeDigitPosition++) {
            for (animeCurrent = 0; animeCurrent < animeMax; animeCurrent++) {
                for (uint8_t position = DIGIT_COUNT; position > animeDigitPosition-1; position--) {
                    for (uint8_t digit = DIGIT_COUNT; digit > 0; digit--) {
                        data = 0x00;
                        if (digit == position) {
                            data += 0x80;
                        } else {
                            //data = 0x00;
                        }

                        if (digit == animeDigitPosition) {
                            data += anime[animeCurrent];
                        } else if (digit < animeDigitPosition) {
                            data = 0xff;
                        }

                        Tm1637_DisplayBitRowdata(digitdisplay_1, digit-1, data);
                    }
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        }
    }
    vTaskDelete(NULL); // Should never get to here...
*/
/*
// Sample number
    uint8_t listDisp_1[DIGIT_COUNT];
    uint8_t count = 0;
    Tm1637_Init();
    if (Tm1637_Enable(PORT_D9_PIN, PORT_D10_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(PORT_D9_PIN, PORT_D10_PIN, BRIGHT_TYPICAL);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
        vTaskDelete(NULL);
    }
    Tm1637_ClearDisplay(digitdisplay_1);
    while(1){
        if (count == UINT8_MAX) {
            count = 0;
        }
        if (count%2) {
            for (uint8_t i = 0; i < DIGIT_COUNT; i++) {
                listDisp_1[i] = i;
            }
        } else {
            for (uint8_t i = 0; i < DIGIT_COUNT; i++) {
                listDisp_1[i] = i+4;
            }
        }
        Tm1637_DisplayAll(digitdisplay_1, listDisp_1);
        count++;
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    vTaskDelete(NULL); // Should never get to here...
*/
/*
// Sample message
    Tm1637_Init();
    if (Tm1637_Enable(PORT_D9_PIN, PORT_D10_PIN) == ESP_OK) {
        digitdisplay_1 = Tm1637_Attach(PORT_D9_PIN, PORT_D10_PIN, BRIGHT_TYPICAL);
    } else {
        ESP_LOGE(TAG, "Digit Display Tm1637_Enable Error");
        vTaskDelete(NULL);
    }
    while (1) {
        Tm1637_ClearDisplay(digitdisplay_1);
        vTaskDelay(pdMS_TO_TICKS(500));
        Tm1637_DisplayStr(digitdisplay_1, "HELL0-1234567890", 500);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL); // Should never get to here...
*/
}
#endif

#if CONFIG_SOFTWARE_ESP_ULP_TEST
static void init_ulp_program(void)
{
    esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(err);

    /* The first argument is the period index, which is not used by the ULP-RISC-V timer
     * The second argument is the period in microseconds, which gives a wakeup time period of: 20ms
     */
    ulp_set_wakeup_period(0, 20000);

    /* Start the program */
    err = ulp_riscv_run();
    ESP_ERROR_CHECK(err);
}
#endif

void app_main() {
    ESP_LOGI(TAG, "app_main() start.");
    esp_log_level_set("*", ESP_LOG_ERROR);
//    esp_log_level_set("wifi", ESP_LOG_INFO);
//    esp_log_level_set("gpio", ESP_LOG_INFO);
    esp_log_level_set("MY-MAIN", ESP_LOG_INFO);
    esp_log_level_set("MY-UI", ESP_LOG_INFO);
    esp_log_level_set("MY-WIFI", ESP_LOG_INFO);

#ifdef CONFIG_SOFTWARE_ESP_LCD_UI_SUPPORT
#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
#if CONFIG_HARDWARE_MODEL_XIAO_EPS32C3
    vTaskDelay(pdMS_TO_TICKS(10000));
#endif
    // ESP-LCD
    Ssd1306_I2c_Init(I2C_NUM_0, PORT_SDA_PIN, PORT_SCL_PIN, GPIO_NUM_NC, PORT_I2C_STANDARD_BAUD);
    ui_start();
#endif
#endif

#ifdef CONFIG_SOFTWARE_ESP_WIFI_SUPPORT
    Wifi_Init();
//    initialise_wifi();
#endif

#ifdef CONFIG_SOFTWARE_UNIT_LED_SUPPORT
    // EXTERNAL LED
    xTaskCreatePinnedToCore(&external_led_task, "external_led_task", 4096 * 1, NULL, 2, &xExternalLED, 1);
#endif

#ifdef CONFIG_SOFTWARE_UNIT_BUTTON_SUPPORT
    // BUTTON
    xTaskCreatePinnedToCore(&button_task, "button_task", 4096 * 1, NULL, 2, &xButton, 1);
#endif

#ifdef CONFIG_SOFTWARE_UNIT_RTC_SUPPORT
    // rtc
    xTaskCreatePinnedToCore(&vLoopRtcTask, "rtc_task", 4096 * 1, NULL, 2, &xRtc, 1);
    // clock
    xTaskCreatePinnedToCore(&vLoopClockTask, "clock_task", 4096 * 1, NULL, 2, &xClock, 1);
#endif

#if CONFIG_SOFTWARE_UNIT_SK6812_SUPPORT
    // EXTERNAL RGB LED BLINK
    xTaskCreatePinnedToCore(&vexternal_LoopRGBLedBlinkTask, "external_rgb_led_blink_task", 4096 * 1, NULL, 2, &xExternalRGBLedBlink, 1);
#endif

#ifdef CONFIG_SOFTWARE_UNIT_ENV2_SUPPORT
    // ENV2
    xTaskCreatePinnedToCore(&vLoopUnitEnv2Task, "unit_env2_task", 4096 * 1, NULL, 2, &xUnitEnv2, 1);
#endif

#ifdef CONFIG_SOFTWARE_UNIT_ENV3_SUPPORT
    // ENV3
    xTaskCreatePinnedToCore(&vLoopUnitEnv3Task, "unit_env3_task", 4096 * 1, NULL, 2, &xUnitEnv3, 1);
#endif

#ifdef CONFIG_SOFTWARE_I2C_LCD_ST7032_SUPPORT
    // LCD_ST7032
    xTaskCreatePinnedToCore(&vLoopLcdSt7032Task, "lcd_st7032_task", 4096 * 2, NULL, 2, &xLcdSt7032, 1);
#endif

#if ( CONFIG_SOFTWARE_UNIT_4DIGIT_DISPLAY_SUPPORT || CONFIG_SOFTWARE_UNIT_6DIGIT_DISPLAY_SUPPORT )
    // DIGIT DISPLAY
    xTaskCreatePinnedToCore(&vLoopUnitDigitDisplayTask, "vLoopUnitDigitDisplayTask", 4096 * 1, NULL, 2, &xDigitDisplay, 1);
#endif

#ifdef CONFIG_SOFTWARE_BUZZER_SUPPORT
    // BUZZER
    xTaskCreatePinnedToCore(&vLoopBuzzerTask, "buzzer_task", 4096 * 1, NULL, 2, &xBuzzer, 1);
#endif

#if CONFIG_SOFTWARE_ESP_ULP_TEST
    vTaskDelay( pdMS_TO_TICKS(10000) );

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(GPIO_NUM_0);
    rtc_gpio_set_direction(GPIO_NUM_0, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(GPIO_NUM_0);
    rtc_gpio_pullup_dis(GPIO_NUM_0);
    rtc_gpio_hold_en(GPIO_NUM_0);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    /* not a wakeup from ULP, load the firmware */
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not a ULP-RISC-V wakeup, initializing it! \n");
        init_ulp_program();
    }

    /* ULP Risc-V read and detected a change in GPIO_0, prints */
    if (cause == ESP_SLEEP_WAKEUP_ULP) {
        printf("ULP-RISC-V woke up the main CPU! \n");
        printf("ULP-RISC-V read changes in GPIO_0 current is: %s \n",
            (bool)(ulp_gpio_level_previous == 0) ? "Low" : "High" );
    }

    /* Go back to sleep, only the ULP Risc-V will run */
    printf("Entering in deep sleep\n\n");

    /* Small delay to ensure the messages are printed */
    vTaskDelay(100);

    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup());
    esp_deep_sleep_start();
#endif

}