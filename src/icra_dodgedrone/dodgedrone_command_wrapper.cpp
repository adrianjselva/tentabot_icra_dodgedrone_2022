#include "icra_dodgedrone/dodgedrone_command_wrapper.h"

DodgeDroneCommandWrapper::DodgeDroneCommandWrapper(ros::NodeHandle& n) : nh_(n), quadrotor(0.1265158) {
    this->velocity_subscriber = nh_.subscribe("/wrapper/velocity_input", 1, &DodgeDroneCommandWrapper::transformVelocity, this);
    this->command_publisher = nh_.advertise<dodgeros_msgs::Command>("/kingfisher/dodgeros_pilot/feedthrough_command", 1);
    this->quad_state_subscriber = nh_.subscribe("/kingfisher/dodgeros_pilot/state", 1, &DodgeDroneCommandWrapper::updateQuadState, this);

    this->quadrotor.setConstants(4.168863, 4.797071);
}

void DodgeDroneCommandWrapper::transformVelocity(const geometry_msgs::TwistStampedConstPtr &vel) {

}

void DodgeDroneCommandWrapper::updateQuadState(const dodgeros_msgs::QuadStateConstPtr& state) {
    this->quad_state = state;

    this->quadrotor.setAngularVelocities(state->motors[0], state->motors[1], state->motors[2], state->motors[3]);
    CBTR cbtr = this->quadrotor.calculateCBTR();
}