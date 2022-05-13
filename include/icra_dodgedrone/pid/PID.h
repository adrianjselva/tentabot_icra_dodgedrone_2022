//
// Created by adrianselva on 5/12/22.
//

#ifndef TENTABOT_PIDBASE_H
#define TENTABOT_PIDBASE_H

class PID {
private:
    double previousError{};
    double accumulatedError{};
    double accumulatedErrorThreshold{};

    [[nodiscard]] double _proportionalTerm(double error) const;
    [[nodiscard]] double _integralTerm(double error);
    [[nodiscard]] double _derivativeTerm(double error) const;

    double _kP{};
    double _kI{};
    double _kD{};

    double _dT{};
public:
    explicit PID(double kP, double kI, double kD, double iThreshold, double dt);
    explicit PID();
    double calculate(double desired, double actual);
};

#endif //TENTABOT_PIDBASE_H
