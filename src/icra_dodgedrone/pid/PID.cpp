#include "icra_dodgedrone/pid/PID.h"

PID::PID(double kP, double kI, double kD, double iThreshold, double dt) {
    this->_kP = kP;
    this->_kI = kI;
    this->_kD = kD;
    this->accumulatedErrorThreshold = iThreshold;
    this->_dT = dt;

    this->previousError = 0;
    this->accumulatedError = 0;
}

double PID::_proportionalTerm(double error) const {
    return this->_kP * error;
}

double PID::_integralTerm(double error) {
    this->accumulatedError += (error * this->_dT);

    if(this->accumulatedError > this->accumulatedErrorThreshold) {
        this->accumulatedError = 0;
    }

    return this->_kI * accumulatedError;
}

double PID::_derivativeTerm(double error) const {
    double derivative = (error - this->previousError) / this->_dT;
    return this->_kD * derivative;
}

double PID::calculate(double desired, double actual) {
    double error = desired - actual;
    double output = this->_proportionalTerm(error) + this->_integralTerm(error) + this->_derivativeTerm(error);

    this->previousError = error;

    return output;
}

PID::PID() {
    this->accumulatedError = 0;
    this->previousError = 0;
    this->_dT = 0;
    this->accumulatedErrorThreshold = 0;
    this->_kI = 0;
    this->_kD = 0;
    this->_kP = 0;
};
