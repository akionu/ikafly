#pragma once
#include <stdio.h>
#include "pico/stdlib.h"
#include <cstdint>
#include "cam.h"
#include "tjpg.hpp"

class IMGCONE : Cam, TJPGD{
public:
    typedef enum {
        IC_NOGOAL,
        IC_RIGHT,
        IC_SRIGHT,
        IC_FORWARD,
        IC_LEFT,
        IC_SLEFT
    } IMGCONE_RES_T;

    bool init();
    IMGCONE_RES_T capture();
private:
    static int my_out_func(JDEC *jdec, void *bitmap, JRECT *rect);
    void count_vert(uint8_t vert[5], uint8_t area[15]);
    static uint32_t red_area[15];
    uint32_t cnt = 0;
    uint8_t vert[5] = {0};
};
