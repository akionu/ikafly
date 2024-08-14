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
    MODE_FORWARD_CAM,
};

#define SEC2CNT 2

// 初期モード
static uint mode_now = MODE_LANDING;
//static uint mode_now = MODE_GNSS;
//static uint mode_now = MODE_CAM;

const float angle_th = 0.39; //前45度，GPS誘導

// GOAL GPS
// Kyotanabe
//#define GOAL_LAT   34801587
//#define GOAL_LONG 135771023
//
// Noshiro (2024.8.14)
//res: lat: 40.142282, lon: 139.987400
#define GOAL_LAT   (40142282)
#define GOAL_LONG (139987400)

// LANDING THRESHOLD
// FOR TEST
//#define RISE_TH 0.04
//#define FALL_TH 0.05
// FOR FLIGHT
#define RISE_TH (0.25)
#define FALL_TH (0.4)

// 着地判定
#define CNT_LAND_ALTREF (500) // 10 min from boot
#define CNT_LAND_NOALT  (900) // 15 min from boot
