#pragma once
#include <stdio.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"

#include "littlefs.hpp"

class Log : LFS {
    public:
        Log(uint8_t code);
        bool init();
        void showAll();
        bool addLog(int32_t lat, int32_t lon, float dist,
                float yaw, float roll, float pitch, float yaw_goal,// deg -180~180
                int16_t motor_left, int16_t motor_right,
                uint8_t seq, bool stack,
                uint8_t buf[17]);
        void encodeLine(uint8_t dst[32],
                int32_t lat, int32_t lon, float dist,
                float yaw, float roll, float pitch, float yaw_goal,// deg -180~180
                int16_t motor_left, int16_t motor_right,
                uint8_t seq, bool stack,
                uint8_t buf[17]);
        void decodeLine(uint8_t raw[32],
                   uint8_t* code, uint8_t* min, uint8_t* sec,
                   int32_t* lat, int32_t* lon, uint8_t* dist,
                   float* yaw, float* roll, float* pitch, float* yaw_goal,
                   uint8_t* motor_right, uint8_t* motor_left,
                   uint8_t* seq, bool* stack,
                   uint8_t buf[17]);
    private:
        datetime_t time;
        uint8_t code;
        uint8_t wbuf[32];
        lfs_file_t file_log;
};
