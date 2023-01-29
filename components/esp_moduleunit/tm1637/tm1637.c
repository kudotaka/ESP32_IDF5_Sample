// https://github.com/Seeed-Studio/Grove_4Digital_Display
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TM1637.h"
#include "esp_log.h"
#include "string.h"

#define TAG "TM1637"

//  --0x01--
// |        |
//0x20     0x02
// |        |
//  --0x40- -
// |        |
//0x10     0x04
// |        |
//  --0x08--

#define LOW 0
#define HIGH 1

#if CONFIG_SOFTWARE_UNIT_4DIGIT_DISPLAY_SUPPORT
const uint8_t DIGITS = 4; // Number of digits on display
#elif CONFIG_SOFTWARE_UNIT_6DIGIT_DISPLAY_SUPPORT
const uint8_t DIGITS = 6; // Number of digits on display
#endif

static SemaphoreHandle_t digit_display_lock = NULL;

uint8_t char2segments(char c) {
    switch (c) {
        case '_' : return 0x08;
        case '^' : return 0x01; // ¯
        case '-' : return 0x40;
        case '*' : return 0x63; // °
        case ' ' : return 0x00; // space
        case 'A' : return 0x77; // upper case A
        case 'a' : return 0x5f; // lower case a
        case 'B' :              // lower case b
        case 'b' : return 0x7c; // lower case b
        case 'C' : return 0x39; // upper case C
        case 'c' : return 0x58; // lower case c
        case 'D' :              // lower case d
        case 'd' : return 0x5e; // lower case d
        case 'E' :              // upper case E
        case 'e' : return 0x79; // upper case E
        case 'F' :              // upper case F
        case 'f' : return 0x71; // upper case F
        case 'G' :              // upper case G
        case 'g' : return 0x35; // upper case G
        case 'H' : return 0x76; // upper case H
        case 'h' : return 0x74; // lower case h
        case 'I' : return 0x06; // 1
        case 'i' : return 0x04; // lower case i
        case 'J' : return 0x1e; // upper case J
        case 'j' : return 0x16; // lower case j
        case 'K' :              // upper case K
        case 'k' : return 0x75; // upper case K
        case 'L' :              // upper case L
        case 'l' : return 0x38; // upper case L
        case 'M' :              // twice tall n
        case 'm' : return 0x37; // twice tall ∩
        case 'N' :              // lower case n
        case 'n' : return 0x54; // lower case n
        case 'O' :              // lower case o
        case 'o' : return 0x5c; // lower case o
        case 'P' :              // upper case P
        case 'p' : return 0x73; // upper case P
        case 'Q' : return 0x7b; // upper case Q
        case 'q' : return 0x67; // lower case q
        case 'R' :              // lower case r
        case 'r' : return 0x50; // lower case r
        case 'S' :              // 5
        case 's' : return 0x6d; // 5
        case 'T' :              // lower case t
        case 't' : return 0x78; // lower case t
        case 'U' :              // lower case u
        case 'u' : return 0x1c; // lower case u
        case 'V' :              // twice tall u
        case 'v' : return 0x3e; // twice tall u
        case 'W' : return 0x7e; // upside down A
        case 'w' : return 0x2a; // separated w
        case 'X' :              // upper case H
        case 'x' : return 0x76; // upper case H
        case 'Y' :              // lower case y
        case 'y' : return 0x6e; // lower case y
        case 'Z' :              // separated Z
        case 'z' : return 0x1b; // separated Z
    }
    return 0;
}

static uint8_t tube_tab[] = {0x3f, 0x06, 0x5b, 0x4f,
                            0x66, 0x6d, 0x7d, 0x07,
                            0x7f, 0x6f, 0x77, 0x7c,
                            0x39, 0x5e, 0x79, 0x71
                           }; //0~9,A,b,C,d,E,F

void Tm1637_Init() {
    if (digit_display_lock == NULL) {
        digit_display_lock = xSemaphoreCreateMutex();
    }
}

esp_err_t Tm1637_Enable(gpio_num_t clk, gpio_num_t data) {
    esp_err_t ret = ESP_OK;
    ret = Tm1637_PinMode(clk, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Error configuring GPIO %d. Error code: 0x%x.", clk, ret);
    }
    ret = Tm1637_PinMode(data, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Error configuring GPIO %d. Error code: 0x%x.", data, ret);
    }
    return ret;
}

DigitDisplay_t* Tm1637_Attach(gpio_num_t clk, gpio_num_t data, uint8_t brightness) {
    xSemaphoreTake(digit_display_lock, portMAX_DELAY);
    DigitDisplay_t *digitdisplay = (DigitDisplay_t *)malloc(sizeof(DigitDisplay_t) * 1);
    digitdisplay->clk = clk;
    digitdisplay->data = data;
    digitdisplay->brightness = brightness;
    xSemaphoreGive(digit_display_lock);
    return digitdisplay;
}

esp_err_t Tm1637_PinMode(gpio_num_t pin, gpio_mode_t mode) {
    esp_err_t err = ESP_OK;
    if (mode == GPIO_MODE_OUTPUT || mode == GPIO_MODE_INPUT) {
        err = gpio_reset_pin(pin);
        if (err != ESP_OK){
            ESP_LOGE(TAG, "Error gpio_reset_pin GPIO %d. Error code: 0x%x.", pin, err);
        }
        err = gpio_set_direction(pin, mode);
        if (err != ESP_OK){
            ESP_LOGE(TAG, "Error configuring GPIO %d MODE %d. Error code: 0x%x.", pin, mode, err);
        }
    }
    return err;
}

uint8_t Tm1637_PinRead(gpio_num_t pin) {
    return gpio_get_level(pin);
}

esp_err_t Tm1637_PinWrite(gpio_num_t pin, bool level) {
    esp_err_t err = gpio_set_level(pin, level);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Error setting GPIO %d state. Error code: 0x%x.", pin, err);
    }
    return err;
}

uint8_t Tm1637_WriteByte(DigitDisplay_t* digitdisplay, int8_t wr_data) {
    for (uint8_t i = 0; i < 8; i++) { // Sent 8bit data
        Tm1637_PinWrite(digitdisplay->clk, LOW);

        if (wr_data & 0x01) {
            Tm1637_PinWrite(digitdisplay->data, HIGH);    // LSB first
        } else {
            Tm1637_PinWrite(digitdisplay->data, LOW);
        }

        wr_data >>= 1;
        Tm1637_PinWrite(digitdisplay->clk, HIGH);
    }

    Tm1637_PinWrite(digitdisplay->clk, LOW); // Wait for the ACK
    Tm1637_PinWrite(digitdisplay->data, HIGH);
    Tm1637_PinWrite(digitdisplay->clk, HIGH);
    Tm1637_PinMode(digitdisplay->data, GPIO_MODE_INPUT);

    Tm1637_BitDelay();
    uint8_t ack = Tm1637_PinRead(digitdisplay->data);

    if (ack == 0) {
        Tm1637_PinMode(digitdisplay->data, GPIO_MODE_OUTPUT);
        Tm1637_PinWrite(digitdisplay->data, LOW);
    }

    Tm1637_BitDelay();
    Tm1637_PinMode(digitdisplay->data, GPIO_MODE_OUTPUT);
    Tm1637_BitDelay();

    return ack;
}

// Send start signal to TM1637 (start = when both pins goes low)
void Tm1637_Start(DigitDisplay_t* digitdisplay) {
    Tm1637_PinWrite(digitdisplay->clk, HIGH);
    Tm1637_PinWrite(digitdisplay->data, HIGH);
    Tm1637_PinWrite(digitdisplay->data, LOW);
    Tm1637_PinWrite(digitdisplay->clk, LOW);
}

// End of transmission (stop = when both pins goes high)
void Tm1637_Stop(DigitDisplay_t* digitdisplay) {
    Tm1637_PinWrite(digitdisplay->clk, LOW);
    Tm1637_PinWrite(digitdisplay->data, LOW);
    Tm1637_PinWrite(digitdisplay->clk, HIGH);
    Tm1637_PinWrite(digitdisplay->data, HIGH);
}

void Tm1637_DisplayAll(DigitDisplay_t* digitdisplay, uint8_t disp_data[]) {
    uint8_t seg_data[DIGITS];
    uint8_t i;

    for (i = 0; i < DIGITS; i++) {
        seg_data[i] = disp_data[i];
    }

    Tm1637_Coding_Full(seg_data);
    Tm1637_Start(digitdisplay);              // Start signal sent to TM1637 from MCU
    Tm1637_WriteByte(digitdisplay, ADDR_AUTO); // Command1: Set data
    Tm1637_Stop(digitdisplay);
    Tm1637_Start(digitdisplay);
    Tm1637_WriteByte(digitdisplay, STARTADDR); // Command2: Set address (automatic address adding)

    for (i = 0; i < DIGITS; i++) {
        Tm1637_WriteByte(digitdisplay, seg_data[i]);    // Transfer display data (8 bits x num_of_digits)
    }

    Tm1637_Stop(digitdisplay);
    Tm1637_Start(digitdisplay);
    Tm1637_WriteByte(digitdisplay, CMD_DISP_CTRL_BASE + digitdisplay->brightness); // Control display
    Tm1637_Stop(digitdisplay);
}

void Tm1637_DisplayBit(DigitDisplay_t* digitdisplay, uint8_t bit_addr, uint8_t disp_data) {
    Tm1637_DisplayBitAddPoint(digitdisplay, bit_addr, disp_data, POINT_OFF);
}

void Tm1637_DisplayBitAddPoint(DigitDisplay_t* digitdisplay, uint8_t bit_addr, uint8_t disp_data, uint8_t add_point) {
    uint8_t seg_data;

    seg_data = Tm1637_Coding_One(disp_data, add_point);
    Tm1637_Start(digitdisplay);               // Start signal sent to TM1637 from MCU
    Tm1637_WriteByte(digitdisplay, ADDR_FIXED); // Command1: Set data
    Tm1637_Stop(digitdisplay);
    Tm1637_Start(digitdisplay);
    Tm1637_WriteByte(digitdisplay, bit_addr | STARTADDR); // Command2: Set data (fixed address)
    Tm1637_WriteByte(digitdisplay, seg_data);        // Transfer display data 8 bits
    Tm1637_Stop(digitdisplay);
    Tm1637_Start(digitdisplay);
    Tm1637_WriteByte(digitdisplay, CMD_DISP_CTRL_BASE + digitdisplay->brightness); // Control display
    Tm1637_Stop(digitdisplay);
}

void Tm1637_DisplayBitRowdata(DigitDisplay_t* digitdisplay, uint8_t bit_addr, uint8_t row_data) {
    Tm1637_Start(digitdisplay);               // Start signal sent to TM1637 from MCU
    Tm1637_WriteByte(digitdisplay, ADDR_FIXED); // Command1: Set data
    Tm1637_Stop(digitdisplay);
    Tm1637_Start(digitdisplay);
    Tm1637_WriteByte(digitdisplay, bit_addr | STARTADDR); // Command2: Set data (fixed address)
    Tm1637_WriteByte(digitdisplay, row_data);        // Transfer display data 8 bits
    Tm1637_Stop(digitdisplay);
    Tm1637_Start(digitdisplay);
    Tm1637_WriteByte(digitdisplay, CMD_DISP_CTRL_BASE + digitdisplay->brightness); // Control display
    Tm1637_Stop(digitdisplay);
}

void Tm1637_DisplayStr(DigitDisplay_t* digitdisplay, char str[], uint16_t loop_delay) {
		int end = strlen(str);
		if(end <= DIGITS) {
			for (int i = 0; i < DIGITS; i++) {
				if (i<0 || i>=end) { // display nothing on the remaining display
                    Tm1637_DisplayBit(digitdisplay, i, DATA_CREAR);
				}
				else {
                	Tm1637_DisplayBit(digitdisplay, i, str[i]);
                }
    	    }
		}
		else {
			int offset=-DIGITS;

			for (int i = 0; i <= end+DIGITS; i++) {
				for (int j = offset, k=0; j < DIGITS+offset; j++,k++) {
					if (j<0 || j>=end) {
						Tm1637_DisplayBit(digitdisplay, k, DATA_CREAR);
					}
					else {
                		Tm1637_DisplayBit(digitdisplay, k, str[j]);
        	        }
    		    }
    		    offset++;
                vTaskDelay(pdMS_TO_TICKS(loop_delay)); //loop delay
			}
		}
}

void Tm1637_ClearDisplay(DigitDisplay_t* digitdisplay) {
#if CONFIG_SOFTWARE_UNIT_4DIGIT_DISPLAY_SUPPORT
    uint8_t clear[] = {DATA_CREAR, DATA_CREAR, DATA_CREAR, DATA_CREAR};
#elif CONFIG_SOFTWARE_UNIT_6DIGIT_DISPLAY_SUPPORT
    uint8_t clear[] = {DATA_CREAR, DATA_CREAR, DATA_CREAR, DATA_CREAR, DATA_CREAR, DATA_CREAR};
#endif
    Tm1637_DisplayAll(digitdisplay, clear);
}

void Tm1637_Coding_Full(uint8_t disp_data[]) {
    for (uint8_t i = 0; i < DIGITS; i++) {
        disp_data[i] = Tm1637_Coding_One(disp_data[i], POINT_OFF);
    }
}

uint8_t Tm1637_Coding_One(uint8_t disp_data, uint8_t add_point) {
    if (disp_data == 0x7f) {
        disp_data = 0x00;    // Clear digit
    } else if (disp_data < (sizeof(tube_tab) / sizeof(*tube_tab))) {
        disp_data = tube_tab[disp_data];
    } else if (disp_data >= '0' && disp_data <= '9') {
        disp_data = tube_tab[(disp_data) - 48];    // char to int (char "0" = ASCII 48)
    } else {
        disp_data = char2segments(disp_data);
    }

    if (add_point == POINT_ON) {
        disp_data += 0x80;
    }

    return disp_data;
}

void Tm1637_BitDelay(void) {
    vTaskDelay(pdMS_TO_TICKS(5));
}
