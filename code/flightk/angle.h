#pragma once
#include <math.h>

class Angle {
    public:
        typedef struct {
            float st;
            float ct;
        } angle_t;
        angle_t constrain(float angle);
        angle_t plus(angle_t q1, angle_t q2);
        angle_t conj(angle_t q);
        angle_t minus(angle_t q1, angle_t q2);
        angle_t theta(angle_t target, angle_t current);
};
