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
//static uint mode_now = MODE_LANDING;
static uint mode_now = MODE_GNSS;
//static uint mode_now = MODE_CAM;

const float angle_th = 0.39; //前45度，GPS誘導


#define GOAL_LAT   34802945
#define GOAL_LONG 135771111
#define RISE_TH 0.04
#define FALL_TH 0.05

