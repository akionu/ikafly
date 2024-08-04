#pragma once
#include <stdio.h>
#include "pico/stdlib.h"

enum {
    MODE_WAIT = 1,
    MODE_LANDING = 2,
    MODE_GNSS = 3,
    MODE_CAM = 4,
    MODE_GOAL = 5,
    MODE_SHOWLOG = 9,
    MODE_NICHROME = 10,
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
