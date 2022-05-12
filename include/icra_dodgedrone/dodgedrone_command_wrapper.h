//
// Created by adrianselva on 5/10/22.
//

#ifndef TENTABOT_DODGEDRONE_COMMAND_WRAPPER_H
#define TENTABOT_DODGEDRONE_COMMAND_WRAPPER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <dodgeros_msgs/Command.h>
#include <dodgeros_msgs/QuadState.h>

#include "icra_dodgedrone/model/Quadrotor.h"

class DodgeDroneCommandWrapper {
private:
    Quadrotor quadrotor;

    ros::NodeHandle& nh_;

    ros::Subscriber velocity_subscriber;
    ros::Subscriber quad_state_subscriber;
    ros::Publisher command_publisher;

    dodgeros_msgs::QuadStateConstPtr quad_state;

    void transformVelocity(const geometry_msgs::TwistStampedConstPtr& vel);
    void updateQuadState(const dodgeros_msgs::QuadStateConstPtr& state);
public:
    explicit DodgeDroneCommandWrapper(ros::NodeHandle& n);
};

#endif //TENTABOT_DODGEDRONE_COMMAND_WRAPPER_H
