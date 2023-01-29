/*
 * AWS IoT EduKit - Core2 for AWS IoT EduKit
 * Cloud Connected Blinky v1.4.1
 * wifi.c
 * 
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <string.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#ifdef CONFIG_OTA_FIRMEARE_UPGRADE_SUPPORT
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#ifdef CONFIG_OTA_USE_CERT_BUNDLE
#include "esp_crt_bundle.h"
#endif
#endif
#include "nvs_flash.h"

#include "wifi.h"
#ifdef CONFIG_SOFTWARE_ESP_LCD_UI_SUPPORT
#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
#include "i2c_ssd1306.h"
#endif
#endif

static const char *TAG = "MY-WIFI";

EventGroupHandle_t _wifi_event_group;

#ifdef CONFIG_SOFTWARE_ESP_WIFI_SUPPORT
#ifdef CONFIG_OTA_FIRMEARE_UPGRADE_SUPPORT
#define HASH_LEN 32

#ifndef CONFIG_OTA_USE_CERT_BUNDLE
extern const uint8_t server_cert_pem_start[] asm("_binary_cacert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_cacert_pem_end");
#endif

#define OTA_URL_SIZE 256
#define OTA_APP_VERSION "03"
#define OTA_FIRMWARE_UPGRADE_URL CONFIG_OTA_FIRMWARE_UPGRADE_URL "." OTA_APP_VERSION

void self_test()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            if (1) {
                // When self-test was passed //
                ESP_LOGI(TAG, "Self test was passed");
                esp_ota_mark_app_valid_cancel_rollback();
            }
            else {
                // When self-test was failed //
                ESP_LOGE(TAG, "Self test was failed !!!!!");
                ESP_LOGE(TAG, "Will be rollback & reboot !!!!!");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
#if 0
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        break;
#endif
    }
    return ESP_OK;
}

void simple_ota_example_task()
{
    ESP_LOGI(TAG, "Starting OTA example task");
    esp_http_client_config_t config = {
        .url = OTA_FIRMWARE_UPGRADE_URL,
#ifdef CONFIG_OTA_USE_CERT_BUNDLE
        .crt_bundle_attach = esp_crt_bundle_attach,
#else
        .cert_pem = (char *)server_cert_pem_start,
#endif /* CONFIG_OTA_USE_CERT_BUNDLE */
        .event_handler = _http_event_handler,
        .keep_alive_enable = true,
    };

#ifdef CONFIG_OTA_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif
/*
    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
    esp_err_t ret = esp_https_ota(&ota_config);
*/
    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s %s", label, hash_print);
}

static void get_sha256_of_partitions(void)
{
    uint8_t sha_256[HASH_LEN] = { 0 };
    esp_partition_t partition;

    // get sha256 digest for bootloader
    partition.address   = ESP_BOOTLOADER_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_OFFSET;
    partition.type      = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");
}

void simple_ota_task()
{
    self_test();

    ESP_LOGI(TAG, "Hello world! (Ver.%s)", OTA_APP_VERSION);
    vTaskDelay( pdMS_TO_TICKS(10000) );

    simple_ota_example_task();
}
#endif

esp_err_t wifi_isConnected(void) {
    EventBits_t status = xEventGroupGetBits(_wifi_event_group);
    if ((status & CONNECTED_BIT) && !(status & DISCONNECTED_BIT)) {
        ESP_LOGI(TAG, "connected! %ld", status);
        return ESP_OK;
    } else {
        ESP_LOGI(TAG, "disconnected. %ld", status);
    }
    return ESP_ERR_INVALID_STATE;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGE(TAG, "Wi-Fi disconnected. Reason: %d", event->reason);
        xEventGroupClearBits(_wifi_event_group, CONNECTED_BIT);
        xEventGroupSetBits(_wifi_event_group, DISCONNECTED_BIT);
#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
        ui_wifi_label_update(false);
#endif
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Device IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupClearBits(_wifi_event_group, DISCONNECTED_BIT);
        xEventGroupSetBits(_wifi_event_group, CONNECTED_BIT);
#ifdef CONFIG_SOFTWARE_MODEL_SSD1306_I2C
        ui_wifi_label_update(true);
#endif
    }
//
//    else if (event_base == IP_EVENT && event_id == IP_EVENT_GOT_IP6) {
//        ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
//        ESP_LOGI(TAG, "Device IPv6 address: " IPV6STR, IPV62STR(event->ip6_info.ip));
//    } 
}

void initialise_wifi(void){
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

#ifdef CONFIG_OTA_FIRMEARE_UPGRADE_SUPPORT
    get_sha256_of_partitions();
#endif

    _wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, 0));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, 0));
//    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_GOT_IP6, &wifi_event_handler, NULL, 0));

    // Initialize default station as network interface instance (esp-netif)
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_LOGI(TAG, "Setting Wi-Fi configuration to SSID: %s", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}
void Wifi_Init(void) {
    initialise_wifi();
}
#endif