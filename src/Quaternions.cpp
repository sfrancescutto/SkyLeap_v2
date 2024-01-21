#include "Quaternion.hpp"
#include "math.h"

Quaternion::Quaternion(){
    this->q0 = 0;
    this->q1 = 0;
    this->q2 = 0;
    this->q3 = 0;
}

Quaternion::Quaternion(double q0, double q1, double q2, double q3){
    this->q0 = q0;
    this->q1 = q1;
    this->q2 = q2;
    this->q3 = q3;
}

void Quaternion::fromRPY(double r, double p, double y){
    double q0,q1,q2,q3;
    y /=2;
    p /=2;
    r /=2;
    double cr = cos(r);
    double cp = cos(p);
    double cy = cos(y);
    double sr = sin(r);
    double sp = sin(p);
    double sy = sin(y);

    this->q0 = cr*cp*cy + sr*sp*sy;
    this->q1 = sr*cp*cy - cr*sp*sy;
    this->q2 = cr*sp*cy + sr*cp*sy;
    this->q3 = cr*cp*sy - sr*sp*cy;
}

Quaternion Quaternion::invert() {
    return Quaternion(this->q0,-this->q1,-this->q2,-this->q3);
    // Quaternion invertedQuaternion(this->q0,-this->q1,-this->q2,-this->q3);
    // return invertedQuaternion;
}

Quaternion Quaternion::operator*(const Quaternion& B){
    Quaternion R;
    R.q0 = (this->q0)*B.q0 - (this->q1)*B.q1 - (this->q2)*B.q2 - (this->q3)*B.q3;
    R.q1 = (this->q0)*B.q1 + (this->q1)*B.q0 - (this->q2)*B.q3 + (this->q3)*B.q2;
    R.q2 = (this->q0)*B.q2 + (this->q1)*B.q3 + (this->q2)*B.q0 - (this->q3)*B.q1;
    R.q3 = (this->q0)*B.q3 - (this->q1)*B.q2 + (this->q2)*B.q1 + (this->q3)*B.q0;

    return R;
}