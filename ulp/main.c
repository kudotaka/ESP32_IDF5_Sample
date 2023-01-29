/* ULP-RISC-V example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   This code runs on ULP-RISC-V  coprocessor
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"
#include "soc/rtc_cntl_reg.h"

static bool gpio_level = false;

// this variable will be exported as a public symbol, visible from main CPU:
bool gpio_level_previous = false;

#define GPIO_LED GPIO_NUM_4

int main (void)
{
    // GPIP
    gpio_level = (bool)ulp_riscv_gpio_get_level(GPIO_NUM_0);
    gpio_level_previous = gpio_level;
    // LED
    ulp_riscv_gpio_init(GPIO_LED);
    ulp_riscv_gpio_output_enable(GPIO_LED);
    ulp_riscv_gpio_set_output_mode(GPIO_LED, RTCIO_MODE_OUTPUT);

    while(1) {
        // GPIO
        gpio_level = (bool)ulp_riscv_gpio_get_level(GPIO_NUM_0);

        // Wakes up the main CPU if pin changed its state
        if(gpio_level != gpio_level_previous) {
            gpio_level_previous = gpio_level;
            ulp_riscv_wakeup_main_processor();
            break;
        }

        //LED
        ulp_riscv_gpio_output_level(GPIO_LED, 1);
        ulp_riscv_delay_cycles(ULP_RISCV_CYCLES_PER_MS * 1000); // wait 1000 ms
        ulp_riscv_gpio_output_level(GPIO_LED, 0);
        ulp_riscv_delay_cycles(ULP_RISCV_CYCLES_PER_MS * 1000);        
    }
    // ulp_riscv_halt() is called automatically when main exits
    return 0;
}