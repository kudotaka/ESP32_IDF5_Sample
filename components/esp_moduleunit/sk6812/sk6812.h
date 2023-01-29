/*
 * This file is part of the MicroPython ESP32 project, https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 LoBo (https://github.com/loboris)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#if BUILD_ESP_IDF_VERSION_5
#else
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "sk6812.h"

#define SK6812_COLOR_OFF     0x000000
#define SK6812_COLOR_BLACK   0x000000
#define SK6812_COLOR_BLUE    0x0000FF
#define SK6812_COLOR_LIME    0x00FF00
#define SK6812_COLOR_AQUA    0x00FFFF
#define SK6812_COLOR_RED     0xFF0000
#define SK6812_COLOR_MAGENTA 0xFF00FF
#define SK6812_COLOR_YELLOW  0xFFFF00
#define SK6812_COLOR_WHITE   0xFFFFFF

typedef struct pixel_timing {
	uint16_t t0h;
	uint16_t t0l;
	uint16_t t1h;
	uint16_t t1l;
	uint32_t reset;
} pixel_timing_t;

typedef struct pixel_settings {
	uint8_t *pixels;		// buffer containing pixel values, 3 (RGB) or 4 (RGBW) bytes per pixel
	pixel_timing_t timings;	// timing data from which the pixels BIT data are formed
	uint16_t pixel_count;	// number of used pixels
	uint8_t brightness;		// brightness factor applied to pixel color
	char color_order[5];
	uint8_t nbits;			// number of bits used (24 for RGB devices, 32 for RGBW devices)
} pixel_settings_t;

void Sk6812_Init(pixel_settings_t *px, int gpioNum, rmt_channel_t channel, uint8_t pixelCount);
void Sk6812_SetColor(pixel_settings_t *px, uint16_t pos, uint32_t color);
void Sk6812_SetAllColor(pixel_settings_t *px, uint32_t color);
void Sk6812_SetBrightness(pixel_settings_t *px, uint8_t brightness);
void Sk6812_Show(pixel_settings_t *px, rmt_channel_t channel);
void Sk6812_Clear(pixel_settings_t *px);

void np_set_pixel_color(pixel_settings_t *px, uint16_t idx, uint32_t color);
void np_set_pixel_color_hsb(pixel_settings_t *px, uint16_t idx, float hue, float saturation, float brightness);
uint32_t np_get_pixel_color(pixel_settings_t *px, uint16_t idx, uint8_t *white);
void np_show(pixel_settings_t *px, rmt_channel_t channel);
void np_clear(pixel_settings_t *px);

int neopixel_init(int gpioNum, rmt_channel_t channel);
void neopixel_deinit(rmt_channel_t channel);

void rgb_to_hsb( uint32_t color, float *hue, float *sat, float *bri );
uint32_t hsb_to_rgb(float hue, float saturation, float brightness);
uint32_t hsb_to_rgb_int(int hue, int sat, int brightness);
#endif
