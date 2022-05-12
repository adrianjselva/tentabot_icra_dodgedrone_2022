#ifndef TENTABOT_QUADROTOR_H
#define TENTABOT_QUADROTOR_H

#include <Eigen/Dense>

struct CBTR {
    double thrust; // (N)
    double roll; // (N-m)
    double pitch; // (N-m)
    double yaw; // (N-m)
};

struct MotorAngularVelocities {
    double motor1; // (rad/s)
    double motor2; // (rad/s)
    double motor3; // (rad/s)
    double motor4; // (rad/s)
};

class Quadrotor {
private:
    CBTR _cbtr;
    MotorAngularVelocities _angularVelocities;

    double _kD;
    double _kT;

    double _armLength; // (m)
public:
    explicit Quadrotor(double armLength);

    CBTR calculateCBTR();
    MotorAngularVelocities calculateMotorAngularVelocities();

    void setAngularVelocities(double motor1, double motor2, double motor3, double motor4);
    void setConstants(double kD, double kT);
};

#endif //TENTABOT_QUADROTOR_H
