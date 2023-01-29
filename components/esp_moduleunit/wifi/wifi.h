/*
 * AWS IoT EduKit - Core2 for AWS IoT EduKit
 * Cloud Connected Blinky v1.4.1
 * wifi.h
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

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
//EventGroupHandle_t _wifi_event_group;

/* The event group allows multiple bits for each event */
#define CONNECTED_BIT BIT0
#define DISCONNECTED_BIT BIT1

#ifdef CONFIG_SOFTWARE_ESP_WIFI_SUPPORT
esp_err_t wifi_isConnected(void);
void initialise_wifi(void);
void simple_ota_task(void);
void Wifi_Init(void);
#endif