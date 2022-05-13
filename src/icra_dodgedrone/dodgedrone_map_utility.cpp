// LAST UPDATE: 2022.03.23
//
// AUTHOR: Neset Unver Akmandor (NUA)
//
// E-MAIL: akmandor.n@northeastern.edu
//
// DESCRIPTION: TODO...
//
// NUA TODO:
// 1) Map Service is not working with main callback. Asynchronous spin is working but then map gets messy!

// --CUSTOM LIBRARIES--


#include "icra_dodgedrone/dodgedrone_map_utility.h"

DodgeDroneMapUtility::DodgeDroneMapUtility(ros::NodeHandle& n) :
nh_(n),
map_(0.16),
tf_listener(this->tf_buffer) {
    this->pointcloud_subscriber_ = nh_.subscribe("/kingfisher/pointcloud",
                                                 1,
                                                 &DodgeDroneMapUtility::updateMapFromPointcloud,
                                                 this);
    //this->obstacle_pointcloud_subscriber_ = nh_.subscribe("/kingfisher/obstacle_pointcloud",
                                    //                      1,
                                            //              &DodgeDroneMapUtility::updateObstacles,
                                           //               this);
    this->map_publisher_ = nh_.advertise<ufomap_msgs::UFOMapStamped>("/kingfisher/ufomap", 1);

    this->oct = std::make_shared<octomap::ColorOcTree>(0.1);
}

void DodgeDroneMapUtility::publishMap() {
    // This is the UFOMap message object.
    ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
    ufo::map::DepthType pub_depth = 0;
    // Convert UFOMap to ROS message
    if (ufomap_msgs::ufoToMsg(this->map_, msg->map,  true, pub_depth)) {
        // Conversion was successful
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "map";
        this->map_publisher_.publish(msg);

        this->map_.clear();
    }
}

void DodgeDroneMapUtility::updateObstacles(const sensor_msgs::PointCloud2Ptr& obstacle_pointcloud) {
    //this->obstacle_pointcloud = obstacle_pointcloud;
}

void DodgeDroneMapUtility::updateMapFromPointcloud(const sensor_msgs::PointCloud2ConstPtr &pointcloud) {
    // Get transform
    ufo::math::Pose6 transform;
    try {
        // Lookup transform
        geometry_msgs::TransformStamped tf_trans = this->tf_buffer.lookupTransform("map",
                                                                              pointcloud->header.frame_id,
                                                                              ros::Time(0));
        // Convert ROS transform to UFO transform
        transform = ufomap_ros::rosToUfo(tf_trans.transform);
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1, "%s", ex.what());
        return;
    }

    ufo::map::PointCloudColor cloud;
    ufo::map::PointCloudColor obstacle_cloud;
    // Convert ROS point cloud to UFO point cloud
    ufomap_ros::rosToUfo(*pointcloud, cloud);
    //ufomap_ros::rosToUfo(*this->obstacle_pointcloud, obstacle_cloud);
    // Transform point cloud to correct frame, do it in parallel (second param true)
    cloud.transform(transform, true);
    obstacle_cloud.transform(transform, true);

    // Integrate point cloud into UFOMap, no max range (third param -1),
    // free space at depth level 1 (fourth param 1)
    this->map_.insertPointCloudDiscrete(transform.translation(), cloud, 30, 1);
    //this->map_.insertPointCloudDiscrete(transform.translation(), obstacle_cloud, 50, 1);
    this->publishMap();
}

