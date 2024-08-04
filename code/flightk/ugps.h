#pragma once
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

extern "C" {
    #include "nmeap.h"
}

class GPS {
    public:
        GPS(int32_t goal_lat, int32_t goal_long, 
            int16_t lat_per_res, int16_t long_per_res,
            double(*fx)(double));
        void init(uint tx_pin, uint rx_pin);
        bool isReady();
        void calc();
        float getDirection();
        float getDistance();
        int32_t getLatitude();
        int32_t getLongitude();
        int32_t goal_latitude;
        int32_t goal_longitude;

        void setFx(double(*fx)(double));
    private:
        //bool is_ready;
        //void callout(nmeap_context_t *context, void *data, void *user_data);
        //void on_uart1_rx();
        int32_t approxDistance(int32_t dx, int32_t dy);
        nmeap_gga_t gga;
        uint16_t long_per_res;
        uint16_t lat_per_res;
        double (*fx)(double);

        int32_t lat_now;
        int32_t long_now;
        int32_t dx;
        int32_t dy;
};

