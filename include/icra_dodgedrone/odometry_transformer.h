//
// Created by adrianselva on 5/6/22.
//

#ifndef TENTABOT_ODOMETRY_TRANSFORMER_H
#define TENTABOT_ODOMETRY_TRANSFORMER_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class OdometryTransformer {
private:
    ros::NodeHandle& nh_;
    ros::Subscriber odometry_subscriber;
public:
    explicit OdometryTransformer(ros::NodeHandle& n);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
};

#endif //TENTABOT_ODOMETRY_TRANSFORMER_H
