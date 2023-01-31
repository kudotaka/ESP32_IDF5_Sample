#include "i2c_ssd1306.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "i2c_ssd1306.h"

#define SSD1306_ADDR (0x3C)
// The pixel number in horizontal and vertical
#define ESP32_LCD_H_RES              128
#define ESP32_LCD_V_RES              64
// Bit number used to represent command and parameter
#define ESP32_LCD_CMD_BITS           8
#define ESP32_LCD_PARAM_BITS         8
#define ESP32_LCD_LVGL_TICK_PERIOD_MS    2

#if CONFIG_SOFTWARE_ESP_LCD_UI_SUPPORT

#ifdef CONFIG_SOFTWARE_ESP_LCD_UI_SUPPORT
static char *TAG = "MY-UI";
#endif

SemaphoreHandle_t xGuiSemaphore;
static lv_disp_t *disp;
static lv_obj_t *active_screen;

#ifdef CONFIG_SOFTWARE_ESP_WIFI_SUPPORT
static lv_obj_t *wifi_label;
#endif

#if ( CONFIG_SOFTWARE_UNIT_ENV2_SUPPORT || CONFIG_SOFTWARE_UNIT_ENV3_SUPPORT )
#define MOJI_DEGREESIGN  "°C"
static lv_obj_t *humidity_current;
static lv_obj_t *humidity_label;
static lv_obj_t *temperature_current;
static lv_obj_t *temperature_label;
#endif

#ifdef CONFIG_SOFTWARE_UNIT_RTC_SUPPORT
static lv_obj_t *datetime_txtlabel;
#endif

#ifdef CONFIG_SOFTWARE_UNIT_BUTTON_SUPPORT
static lv_obj_t *button_label;
#endif


#ifdef CONFIG_SOFTWARE_ESP_WIFI_SUPPORT
void ui_wifi_label_update(bool state){
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    if (state == false) {
        lv_label_set_text(wifi_label, "");
    } 
    else{
        lv_label_set_text(wifi_label, LV_SYMBOL_WIFI);
    }
    xSemaphoreGive(xGuiSemaphore);
}
#endif

#ifdef CONFIG_SOFTWARE_UNIT_BUTTON_SUPPORT
void ui_button_label_update(bool state){
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    if (state == false) {
        lv_label_set_text(button_label, "");
    } 
    else{
        lv_label_set_text(button_label, LV_SYMBOL_OK);
    }
    xSemaphoreGive(xGuiSemaphore);
}
#endif

#ifdef CONFIG_SOFTWARE_UNIT_RTC_SUPPORT
void ui_datetime_set(char *dateTxt) {
    if( dateTxt != NULL ){
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

        lv_label_set_text(datetime_txtlabel, dateTxt);

        xSemaphoreGive(xGuiSemaphore);
    } 
    else{
        ESP_LOGE(TAG, "datetime dateTxt is NULL!");
    }
}
#endif

#if ( CONFIG_SOFTWARE_UNIT_ENV2_SUPPORT || CONFIG_SOFTWARE_UNIT_ENV3_SUPPORT )
void ui_humidity_update(int32_t value){
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

    lv_label_set_text_fmt(humidity_current, "%ld", value);

    xSemaphoreGive(xGuiSemaphore);
}

void ui_temperature_update(int32_t value){
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

    lv_label_set_text_fmt(temperature_current, "%ld", value);

    xSemaphoreGive(xGuiSemaphore);
}
#endif

void ui_start()
{
#ifdef CONFIG_SOFTWARE_ESP_LCD_UI_SUPPORT
    ESP_LOGI(TAG, "ui_start() start.");

    if (xGuiSemaphore == NULL) {
        xGuiSemaphore = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

    active_screen = lv_disp_get_scr_act(disp);

#ifdef CONFIG_SOFTWARE_ESP_WIFI_SUPPORT
    wifi_label = lv_label_create(active_screen);
    lv_obj_align(wifi_label, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_label_set_text(wifi_label, "");
#endif

#ifdef CONFIG_SOFTWARE_UNIT_BUTTON_SUPPORT
    button_label = lv_label_create(active_screen);
    lv_obj_align(button_label, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_label_set_text(button_label, "");
#endif

#if ( CONFIG_SOFTWARE_UNIT_ENV2_SUPPORT || CONFIG_SOFTWARE_UNIT_ENV3_SUPPORT )
    humidity_current = lv_label_create(active_screen);
    lv_label_set_long_mode(humidity_current, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(humidity_current, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text_fmt(humidity_current, "%d", 0);
    lv_obj_align(humidity_current, LV_ALIGN_TOP_LEFT, 0, 16);
    humidity_label = lv_label_create(active_screen);
    lv_obj_set_style_text_align(humidity_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(humidity_label, "%");
    lv_obj_align(humidity_label, LV_ALIGN_TOP_LEFT, 27, 16);

    temperature_current = lv_label_create(active_screen);
    lv_label_set_long_mode(temperature_current, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(temperature_current, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text_fmt(temperature_current, "%d", 0);
    lv_obj_align(temperature_current, LV_ALIGN_TOP_LEFT, 0, 32);
    temperature_label = lv_label_create(active_screen);
    lv_obj_set_style_text_align(temperature_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(temperature_label, MOJI_DEGREESIGN);
    lv_obj_align(temperature_label, LV_ALIGN_TOP_LEFT, 25, 32);
#endif

#ifdef CONFIG_SOFTWARE_UNIT_RTC_SUPPORT
    char str1[30] = {0};
    sprintf(str1,"%04d/%02d/%02d %02d:%02d:%02d", 2022, 1, 1, 12, 0, 0);
    datetime_txtlabel = lv_label_create(active_screen);
    lv_obj_set_style_text_align(datetime_txtlabel, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(datetime_txtlabel, str1);
    lv_obj_align(datetime_txtlabel, LV_ALIGN_TOP_LEFT, 0, 48);
#endif

    xSemaphoreGive(xGuiSemaphore);
#endif

}


//=======================================================
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void lvgl_set_px_cb(lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                                   lv_color_t color, lv_opa_t opa)
{
    uint16_t byte_index = x + (( y >> 3 ) * buf_w);
    uint8_t  bit_index  = y & 0x7;

    if ((color.full == 0) && (LV_OPA_TRANSP != opa)) {
        buf[byte_index] |= (1 << bit_index);
    } else {
        buf[byte_index] &= ~(1 << bit_index);
    }
}

static void lvgl_rounder(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    area->y1 = area->y1 & (~0x7);
    area->y2 = area->y2 | 0x7;
}

static void lvgl_tick_task(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(ESP32_LCD_LVGL_TICK_PERIOD_MS);
}

static void gui_task(void *pvParameter) {

    while (1) {
        // Delay 1 tick (assumes FreeRTOS tick is 10ms
        vTaskDelay(pdMS_TO_TICKS(10));

        // Try to take the semaphore, call lvgl related function on success
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }

    // A task should NEVER return
    vTaskDelete(NULL);
}

void Ssd1306_I2c_Init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, gpio_num_t rst, uint32_t baud)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = baud,
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = SSD1306_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
        .lcd_cmd_bits = ESP32_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = ESP32_LCD_CMD_BITS, // According to SSD1306 datasheet
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)i2c_num, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = rst,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL library");
    if (xGuiSemaphore == NULL) {
        xGuiSemaphore = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = malloc(ESP32_LCD_H_RES * 20 * sizeof(lv_color_t));
    assert(buf1);
    lv_color_t *buf2 = malloc(ESP32_LCD_H_RES * 20 * sizeof(lv_color_t));
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, ESP32_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ESP32_LCD_H_RES;
    disp_drv.ver_res = ESP32_LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.rounder_cb = lvgl_rounder;
    disp_drv.set_px_cb = lvgl_set_px_cb;
    disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_task,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, ESP32_LCD_LVGL_TICK_PERIOD_MS * 1000));

    xSemaphoreGive(xGuiSemaphore);

    xTaskCreatePinnedToCore(gui_task, "gui_task", 4096 * 2, NULL, 2, NULL, 1);
}
#endif