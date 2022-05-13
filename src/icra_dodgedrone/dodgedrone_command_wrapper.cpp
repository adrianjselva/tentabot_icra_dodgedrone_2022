#include "icra_dodgedrone/dodgedrone_command_wrapper.h"

DodgeDroneCommandWrapper::DodgeDroneCommandWrapper(ros::NodeHandle& n) : nh_(n), quadrotor(0.1265158) {
    this->velocity_subscriber = nh_.subscribe("/wrapper/velocity_input", 1, &DodgeDroneCommandWrapper::transformVelocity, this);
    this->command_publisher = nh_.advertise<dodgeros_msgs::Command>("/kingfisher/dodgeros_pilot/feedthrough_command", 1);
    this->quad_state_subscriber = nh_.subscribe("/kingfisher/dodgeros_pilot/state", 1, &DodgeDroneCommandWrapper::updateQuadState, this);
    this->position_subscriber = nh_.subscribe("/wrapper/position_input", 1, &DodgeDroneCommandWrapper::sendPosition, this);

    this->quadrotor.setConstants(4.168863, 0.00000208);

    double rollKp;
    double rollKi;
    double rollKd;
    double pitchKp;
    double pitchKi;
    double pitchKd;
    double yawKp;
    double yawKi;
    double yawKd;
    double zKp;
    double zKi;
    double zKd;

    double rollRateKp;
    double rollRateKi;
    double rollRateKd;
    double pitchRateKp;
    double pitchRateKi;
    double pitchRateKd;

    double angularXKp;
    double angularXKi;
    double angularXKd;

    double angularYKp;
    double angularYKi;
    double angularYKd;

    double angularZKp;
    double angularZKi;
    double angularZKd;

    nh_.getParam("roll_kP", rollKp);
    nh_.getParam("roll_kI", rollKi);
    nh_.getParam("roll_kD", rollKd);
    nh_.getParam("pitch_kP", pitchKp);
    nh_.getParam("pitch_kI", pitchKi);
    nh_.getParam("pitch_kD", pitchKd);
    nh_.getParam("yaw_kP", yawKp);
    nh_.getParam("yaw_kI", yawKi);
    nh_.getParam("yaw_kD", yawKd);

    nh_.getParam("rollRate_kP", rollRateKp);
    nh_.getParam("rollRate_kI", rollRateKi);
    nh_.getParam("rollRate_kD", rollRateKd);
    nh_.getParam("pitchRate_kP", pitchRateKp);
    nh_.getParam("pitchRate_kI", pitchRateKi);
    nh_.getParam("pitchRate_kD", pitchRateKd);

    nh_.getParam("angularX_kP", angularXKp);
    nh_.getParam("angularX_kI", angularXKi);
    nh_.getParam("angularX_kD", angularXKd);
    nh_.getParam("angularY_kP", angularYKp);
    nh_.getParam("angularY_kI", angularYKi);
    nh_.getParam("angularY_kD", angularYKd);
    nh_.getParam("angularZ_kP", angularZKp);
    nh_.getParam("angularZ_kI", angularZKi);
    nh_.getParam("angularZ_kD", angularZKd);

    nh_.getParam("z_kP", zKp);
    nh_.getParam("z_kI", zKi);
    nh_.getParam("z_kD", zKd);


    this->roll = PID(rollKp, rollKi, rollKd,   100000, 0.1);
    this->pitch = PID(pitchKp, pitchKi, pitchKd, 100000, 0.1);
    this->yaw = PID(yawKp, yawKi, yawKd, 100000, 0.01);
    this->rollRate = PID(rollRateKp, rollRateKi, rollRateKd, 100000, 0.01);
    this->pitchRate = PID(pitchRateKp, pitchRateKi, pitchRateKd, 100000, 0.01);
    this->angularX = PID(angularXKp, angularXKi, angularXKd, 100000, 0.01);
    this->angularY = PID(angularYKp, angularYKi, angularYKd, 100000, 0.01);
    this->angularZ = PID(angularZKp, angularZKi, angularZKd, 100000, 0.01);
    this->z = PID(zKp, zKi, zKd, 100000, 0.1);
}

void DodgeDroneCommandWrapper::transformVelocity(const geometry_msgs::TwistStampedConstPtr &vel) {

}

void DodgeDroneCommandWrapper::sendPosition(const geometry_msgs::PoseStampedConstPtr& pos) {
    double currentX = this->quad_state->pose.position.x;
    double currentY = this->quad_state->pose.position.y;
    double currentZ = this->quad_state->pose.position.z;

    double currentRoll = this->quad_state->pose.orientation.x;
    double currentPitch = this->quad_state->pose.orientation.y;
    double currentYaw = this->quad_state->pose.orientation.z;

    double currentRollRate = this->quad_state->velocity.angular.x;
    double currentPitchRate = this->quad_state->velocity.angular.y;
    double currentYawRate = this->quad_state->velocity.angular.z;

    double desiredX = pos->pose.position.x;
    double desiredY = pos->pose.position.y;
    double desiredZ = pos->pose.position.z;

    double desiredPitch = this->pitch.calculate(desiredX, currentX);
    double desiredRoll = this->roll.calculate(-desiredY, -currentY);
    double desiredYaw = pos->pose.orientation.z;

    double desiredRollRate = this->rollRate.calculate(desiredRoll, currentRoll);
    double desiredPitchRate = this->pitchRate.calculate(desiredPitch, currentPitch);
    double desiredYawRate = this->yaw.calculate(desiredYaw, currentYaw);
    double z_pid = this->z.calculate(desiredZ, currentZ);

    double quad_weight = 7.3696;
    double thrust = (z_pid + quad_weight) / (cos(currentRoll) * cos(currentPitch));

    double outputRoll = this->angularX.calculate(desiredRollRate, currentRollRate);
    double outputPitch = this->angularY.calculate(desiredPitchRate, currentPitchRate);
    double outputYaw = this->angularZ.calculate(desiredYawRate, currentYawRate);

    dodgeros_msgs::CommandPtr cmd = dodgeros_msgs::CommandPtr(new dodgeros_msgs::Command);
    std_msgs::Header header;
    geometry_msgs::Vector3 bodyrates;
    bodyrates.x = outputRoll;
    bodyrates.y = outputPitch;
    bodyrates.z = outputYaw;
    header.stamp = ros::Time::now();
    cmd->t = this->quad_state->t;
    cmd->is_single_rotor_thrust = false;
    cmd->header = header;
    cmd->bodyrates = bodyrates;
    cmd->collective_thrust = thrust;

    this->command_publisher.publish(cmd);
}

void DodgeDroneCommandWrapper::updateQuadState(const dodgeros_msgs::QuadStateConstPtr& state) {
    this->quad_state = state;

    this->quadrotor.setAngularVelocities(state->motors[0], state->motors[1], state->motors[2], state->motors[3]);
    CBTR cbtr = this->quadrotor.calculateCBTR();


}