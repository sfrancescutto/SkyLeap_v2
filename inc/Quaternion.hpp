#pragma once

class Quaternion{
public:
    double q0;
    double q1;
    double q2;
    double q3;

    Quaternion();
    Quaternion(double q0, double q1, double q2, double q3);

    void fromRPY(double roll, double pitch, double yaw);
    Quaternion invert();
    Quaternion operator*(const Quaternion& B);
};