//
// Created by adrianselva on 5/6/22.
//

#ifndef TENTABOT_DODGEDRONE_MAP_UTLITY_H
#define TENTABOT_DODGEDRONE_MAP_UTLITY_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <ufomap_msgs/UFOMapStamped.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufomap_ros/conversions.h>
#include <ufomap_msgs/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <octomap/ColorOcTree.h>


class DodgeDroneMapUtility {
private:
    ros::NodeHandle& nh_;

    ros::Publisher map_publisher_;
    ros::Publisher octomap_publisher_;
    ros::Subscriber pointcloud_subscriber_;

    // TF listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    ufo::map::OccupancyMapColor map_;
    std::shared_ptr<octomap::ColorOcTree> oct;
public:

    explicit DodgeDroneMapUtility(ros::NodeHandle& n);
    void updateMapFromPointcloud(const sensor_msgs::PointCloud2ConstPtr& pointcloud);
    void publishMap();
    void publishOctoMap();
};

#endif //TENTABOT_DODGEDRONE_MAP_UTLITY_H
