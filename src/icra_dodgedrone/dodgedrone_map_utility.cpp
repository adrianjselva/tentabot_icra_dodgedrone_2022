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
map_(0.32),
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

    nh_.getParam("bbx_resolution", bbxMapResolution);
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
    ufo::math::Pose6 worldTransform;
    try {
        // Lookup transform
        geometry_msgs::TransformStamped tf_trans = this->tf_buffer.lookupTransform("map",
                                                                              pointcloud->header.frame_id,
                                                                              ros::Time(0));
        geometry_msgs::TransformStamped tf_world_trans = this->tf_buffer.lookupTransform("map", "world", ros::Time(0));
        // Convert ROS transform to UFO transform
        transform = ufomap_ros::rosToUfo(tf_trans.transform);
        worldTransform = ufomap_ros::rosToUfo(tf_world_trans.transform);
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1, "%s", ex.what());
        return;
    }

    ufo::map::PointCloud cloud;
    ufo::map::PointCloud bounds;



    for(int x = -5; x <= 65; x++) {
        for(int z = 0; z <= 10; z++) {
            double factor = 0.25;

            for(int i = 0; i < (int)(1.0 / factor); i++) {
                for(int j = 0; j < (int)(1.0 / factor); j++) {
                    ufo::map::Point3 left((double) x + factor * i, -10.0, (double) z + factor * j);
                    ufo::map::Point3 right((double) x + factor * i, 10.0, (double) z + factor * j);
                    bounds.push_back(left);
                    bounds.push_back(right);
                }
            }
        }
    }

    for(int x = -5; x <= 65; x++) {
        for (int y = -10; y <= 10; y++) {
            double factor = 0.25;

            for (int i = 0; i < (int) (1.0 / factor); i++) {
                for (int j = 0; j < (int) (1.0 / factor); j++) {
                    ufo::map::Point3 top((double) x + factor * i, (double) y + factor * j, 10.0);
                    bounds.push_back(top);
                }
            }
        }
    }

    // Convert ROS point cloud to UFO point cloud
    ufomap_ros::rosToUfo(*pointcloud, cloud);
    //ufomap_ros::rosToUfo(*this->obstacle_pointcloud, obstacle_cloud);
    // Transform point cloud to correct frame, do it in parallel (second param true)
    cloud.transform(transform, true);
    bounds.transform(worldTransform, true);

    // Integrate point cloud into UFOMap, no max range (third param -1),
    // free space at depth level 1 (fourth param 1)
    this->map_.insertPointCloudDiscrete(transform.translation(), cloud, 30, 1);
    this->map_.insertPointCloudDiscrete(worldTransform.translation(), bounds, -1, 1);
    //this->map_.insertPointCloudDiscrete(transform.translation(), obstacle_cloud, 50, 1);

    this->publishMap();
}


