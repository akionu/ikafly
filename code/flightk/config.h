#pragma once
#include <stdio.h>
#include "pico/stdlib.h"

enum {
    MODE_WAIT,
    MODE_LANDING,
    MODE_GNSS,
    MODE_CAM,
    MODE_GOAL,
    MODE_SHOWLOG,
    MODE_NICHROME,
};

#define SEC2CNT 2

// 初期モード
static uint mode_now = MODE_LANDING;
//static uint mode_now = MODE_GNSS;
//static uint mode_now = MODE_CAM;

// 機体
//#define A
//#define B

const float angle_th = 0.39; //前45度，GPS誘導
