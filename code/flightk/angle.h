#pragma once
#include <math.h>

typedef struct {
    float st;
    float ct;
} angle_t;

class Angle {
    public:
        static void deg2vec(float angle, angle_t* res);
        static void plus(angle_t q1, angle_t q2, angle_t* res);
        static void conj(angle_t q, angle_t* res);
        static void minus(angle_t q1, angle_t q2, angle_t* res);
        static float theta(angle_t target, angle_t current);
        static float theta(float target, float current);
    private:
};
