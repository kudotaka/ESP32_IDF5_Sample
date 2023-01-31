#include "esp_log.h"
#include "driver/gpio.h"

void Buzzer_Init(gpio_num_t pin);
void Buzzer_Play();
void Buzzer_Stop();
void Buzzer_Play_Duty(uint32_t duty);
void Buzzer_Play_Duty_Frequency(uint32_t duty, uint32_t frequency);
