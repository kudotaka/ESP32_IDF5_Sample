
#pragma once
#include "stdint.h"

typedef struct _rtc_data_t {
    uint16_t year;  //Date year.
    uint8_t month;  //Date month.
    uint8_t day;    //Date day.
    uint8_t hour;   //Time hour.
    uint8_t minute; //Time minute.
    uint8_t second; //Time second.
} rtc_date_t;
