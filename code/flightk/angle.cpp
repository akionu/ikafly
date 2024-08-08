#include "angle.h"
//https://akehi.github.io/containts/20161119_angle/anglelib.html

// degree -> angle vector
void Angle::deg2vec(float angle, angle_t* res) {
    res->ct = cos(angle);
    res->st = sin(angle);
}

// q1+q2
void Angle::plus(angle_t q1, angle_t q2, angle_t* res) {
    res->ct = q2.ct*q1.ct-q2.st*q1.st;
    res->st = q2.st*q1.ct+q2.ct*q1.st;
}

// -q
void Angle::conj(angle_t q, angle_t* res) {
    res->ct = q.ct;
    res->st = -q.st;
}

// q1-q2
void Angle::minus(angle_t q1, angle_t q2, angle_t* res) {
    angle_t q2c;
    conj(q2, &q2c);
    plus(q1,q2c, res);
}

float Angle::theta(angle_t target, angle_t current) {
    angle_t delta;
    minus(target, current, &delta);
    return atan2(delta.st, delta.ct);
}

float Angle::theta(float target, float current) {
    angle_t t, c;
    Angle::deg2vec(target, &t);
    Angle::deg2vec(current, &c);
    return (Angle::theta(t, c));
}
