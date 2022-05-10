//
// Created by adrianselva on 5/6/22.
//

#ifndef TENTABOT_POINTCLOUD_GENERATOR_H
#define TENTABOT_POINTCLOUD_GENERATOR_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <envsim_msgs/ObstacleArray.h>
#include <dodgeros_msgs/QuadState.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <utility>
#include <vector>
#include <geometry_msgs/Point32.h>
#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

class PointcloudGenerator {
private:
    image_transport::ImageTransport it_;
    ros::NodeHandle& nh_;

    image_transport::Subscriber depth_subscriber;

    ros::Publisher pointcloud_publisher;
    ros::Publisher camera_info_publisher;
public:
    explicit PointcloudGenerator(ros::NodeHandle& n);

    void processDepth(const sensor_msgs::ImagePtr& depth);

    void publishCameraInfo(const sensor_msgs::CameraInfoPtr& camera_info);
    void publishPointcloud(const sensor_msgs::ImagePtr& depth, const sensor_msgs::CameraInfoPtr& camera_info);

    void transformDepth(const sensor_msgs::ImageConstPtr& depth);

    static sensor_msgs::PointCloud2::Ptr constructPointcloudFromDepth(const sensor_msgs::ImagePtr& depth,
                                                               const sensor_msgs::CameraInfoPtr& camera_info);
    static sensor_msgs::CameraInfoPtr simulatedCameraInfo();
};

#endif //TENTABOT_POINTCLOUD_GENERATOR_H
