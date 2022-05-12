#include "icra_dodgedrone/model/Quadrotor.h"

Quadrotor::Quadrotor(double armLength) : _armLength(armLength) {
    this->_kD = 0;
    this->_kT = 0;

    this->_cbtr = CBTR{};
    this->_angularVelocities = MotorAngularVelocities {};
}

CBTR Quadrotor::calculateCBTR() {
    Eigen::Matrix4d constants;
    constants <<
        _kT, _kT, _kT, _kT,
        0, -_armLength * _kT, 0, _armLength * _kT,
        _armLength * _kT, 0, -_armLength * _kT, 0,
        _kD, -_kD, _kD, -_kD;

    Eigen::Vector4d angularVelocities;
    angularVelocities <<
        _angularVelocities.motor1,
        _angularVelocities.motor2,
        _angularVelocities.motor3,
        _angularVelocities.motor4;

    auto product = constants * angularVelocities;

    _cbtr.thrust = product(0);
    _cbtr.roll = product(1);
    _cbtr.pitch = product(2);
    _cbtr.yaw = product(3);

    return _cbtr;
}

void Quadrotor::setAngularVelocities(double motor1, double motor2, double motor3, double motor4) {
    this->_angularVelocities.motor1 = motor1;
    this->_angularVelocities.motor2 = motor2;
    this->_angularVelocities.motor3 = motor3;
    this->_angularVelocities.motor4 = motor4;
}

void Quadrotor::setConstants(double kD, double kT) {
    this->_kD = kD;
    this->_kT = kT;
}

MotorAngularVelocities Quadrotor::calculateMotorAngularVelocities() {
    Eigen::Vector4d inputs;
    inputs << _cbtr.thrust, _cbtr.roll, _cbtr.pitch, _cbtr.yaw;

    Eigen::Matrix4d constants;
    constants <<
              _kT, _kT, _kT, _kT,
            0, -_armLength * _kT, 0, _armLength * _kT,
            _armLength * _kT, 0, -_armLength * _kT, 0,
            _kD, -_kD, _kD, -_kD;

    auto product = constants.inverse() * inputs;

    _angularVelocities.motor1 = product(0);
    _angularVelocities.motor2 = product(1);
    _angularVelocities.motor3 = product(2);
    _angularVelocities.motor4 = product(3);

    return _angularVelocities;
}
