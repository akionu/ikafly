#pragma once
#include <stdio.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"

#include "littlefs.hpp"

class Log {
    public:
        Log(uint8_t code);
        void init();
        bool addLog(int32_t lat, int32_t lon,
                float yaw, float roll, float pitch,
                uint8_t seq, 
                uint8_t buf[12]);
        void showAll();
        uint32_t head;
        uint8_t bufw[32] = {0};
        
    private:
        datetime_t time;
        uint8_t code;
};
