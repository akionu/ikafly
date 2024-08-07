#include "angle.h"
//https://akehi.github.io/containts/20161119_angle/anglelib.html

// degree -> angle vector
Angle::angle_t Angle::constrain(float angle) {
    angle_t tmp;
    tmp.ct = cos(angle);
    tmp.st = sin(angle);
    return tmp;
}

// q1+q2
Angle::angle_t Angle::plus(angle_t q1, angle_t q2) {
    angle_t plus;
    plus.ct = q2.ct*q1.ct-q2.st*q1.st;
    plus.st = q2.st*q1.ct+q2.ct*q1.st;
    return plus;
}

// -q
Angle::angle_t Angle::conj(angle_t q) {
    angle_t conj;
    conj.ct = q.ct;
    conj.st = -q.st;
    return conj;
}

// q1-q2
Angle::angle_t Angle::minus(angle_t q1, angle_t q2) {
    return (plus(q1,conj(q2)));
}

float Angle::theta(angle_t target, angle_t current) {
    angle_t delta;
    delta = minus(target, current);
    return atan2(delta.st, delta.ct);
}
