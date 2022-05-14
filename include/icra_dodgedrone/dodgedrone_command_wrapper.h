//
// Created by adrianselva on 5/10/22.
//

#ifndef TENTABOT_DODGEDRONE_COMMAND_WRAPPER_H
#define TENTABOT_DODGEDRONE_COMMAND_WRAPPER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <dodgeros_msgs/Command.h>
#include <dodgeros_msgs/QuadState.h>
#include <geometry_msgs/PoseStamped.h>

#include "icra_dodgedrone/model/Quadrotor.h"
#include "icra_dodgedrone/pid/PID.h"

class DodgeDroneCommandWrapper {
private:
    Quadrotor quadrotor;

//    PID roll;
//    PID pitch;
//    PID yaw;
//    PID rollRate;
//    PID pitchRate;
//    PID angularX;
//    PID angularY;
//    PID angularZ;
    PID x;
    PID y;
    PID z;

    PID yaw;

    ros::NodeHandle& nh_;

    ros::Subscriber velocity_subscriber;
    ros::Subscriber position_subscriber;
    ros::Subscriber quad_state_subscriber;
    ros::Publisher command_publisher;

    dodgeros_msgs::QuadStateConstPtr quad_state;

    void transformVelocity(const geometry_msgs::TwistStampedConstPtr& vel);
    void updateQuadState(const dodgeros_msgs::QuadStateConstPtr& state);
    void sendPosition(const geometry_msgs::PoseStampedConstPtr& pos);

public:
    explicit DodgeDroneCommandWrapper(ros::NodeHandle& n);

};

#endif //TENTABOT_DODGEDRONE_COMMAND_WRAPPER_H
