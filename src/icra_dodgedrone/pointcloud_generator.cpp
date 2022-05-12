#include "icra_dodgedrone/pointcloud_generator.h"

PointcloudGenerator::PointcloudGenerator(ros::NodeHandle &n) : nh_(n), it_(n) {
    this->depth_subscriber = it_.subscribe("/kingfisher/dodgeros_pilot/unity/depth",
                                           1,
                                           &PointcloudGenerator::transformDepth,
                                           this);
    this->obstacle_subscriber = nh_.subscribe("/kingfisher/dodgeros_pilot/groundtruth/obstacles",
                                              1,
                                              &PointcloudGenerator::updateObstacles,
                                              this);

    this->camera_info_publisher = nh_.advertise<sensor_msgs::CameraInfo>("/kingfisher/camera_info", 1);
    this->pointcloud_publisher = nh_.advertise<sensor_msgs::PointCloud2>("/kingfisher/pointcloud", 1);
    this->obstacle_pointcloud_publisher = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
            "/kingfisher/obstacle_pointcloud", 1);
}

std::vector<geometry_msgs::Point32> PointcloudGenerator::extractPCLFromObstacle(
        geometry_msgs::Vector3 center, float object_radius)
{
    std::vector<geometry_msgs::Point32> opc;
    float phi = M_PI * (3.0 - sqrt(5.0));

    int samples = 2000;

    for(int i = 0; i < 2000; i++) {
        float y = 1 - ((float)i / (float)(samples - 1)) * 2;
        float radius = sqrt(1 - y * y);

        float theta = phi * (float)i;

        float x = cos(theta) * radius;
        float z = sin(theta) * radius;

        geometry_msgs::Point32 outputPoint;

        outputPoint.x = (x * object_radius) + center.x;
        outputPoint.y = (y * object_radius) + center.y;
        outputPoint.z = (z * object_radius) + center.z;

        opc.push_back(outputPoint);
    }

    return opc;
}

void PointcloudGenerator::processDepth(const sensor_msgs::ImagePtr &depth) {
    depth->header.frame_id = "camera";

    auto camera_info = simulatedCameraInfo();

    this->publishCameraInfo(camera_info);
    this->publishPointcloud(depth, camera_info);
}

void PointcloudGenerator::publishCameraInfo(const sensor_msgs::CameraInfoPtr &msg) {
    this->camera_info_publisher.publish(msg);
}

void PointcloudGenerator::publishPointcloud(const sensor_msgs::ImagePtr &depth,
                                            const sensor_msgs::CameraInfoPtr &camera_info) {
    this->pointcloud_publisher.publish(constructPointcloudFromDepth(depth, camera_info));
}

void PointcloudGenerator::transformDepth(const sensor_msgs::ImageConstPtr &depth) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image = cv_ptr->image * 100.0f;

    this->processDepth(cv_ptr->toImageMsg());
}

sensor_msgs::PointCloud2::Ptr PointcloudGenerator::constructPointcloudFromDepth(const sensor_msgs::ImagePtr &depth,
                                                                                const sensor_msgs::CameraInfoPtr &camera_info) {
    sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
    cloudMsg->header = depth->header;
    cloudMsg->height = depth->height;
    cloudMsg->width = depth->width;
    cloudMsg->is_dense = false;
    cloudMsg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloudMsg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camera_info);

    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth->encoding == sensor_msgs::image_encodings::MONO16) {
        depth_image_proc::convert<uint16_t>(depth, cloudMsg, model);
    } else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        depth_image_proc::convert<float>(depth, cloudMsg, model);
    }

    return cloudMsg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointcloudGenerator::constructPointcloudFromObstacles(const envsim_msgs::ObstacleArrayConstPtr &obstacles) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);
    msg->header.frame_id = "kingfisher";
    msg->height = 1;

    for(auto o : this->obstacles->obstacles) {
        for(auto p : extractPCLFromObstacle(o.position, o.scale)) {
            msg->points.emplace_back(p.x, p.y, p.z);
        }
    }

    msg->width = msg->points.size();

    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);

    return msg;
}

sensor_msgs::CameraInfoPtr PointcloudGenerator::simulatedCameraInfo() {
    auto *camera_info_msg = new sensor_msgs::CameraInfo;
    camera_info_msg->header.frame_id = "camera";
    camera_info_msg->width = 320;
    camera_info_msg->height = 240;
    camera_info_msg->K = {142.82927, 0, 160.0, 0, 142.82927, 120.0, 0, 0, 1};
    camera_info_msg->P = {142.82927, 0, 160.0, 0, 0, 142.82927, 120.0, 0, 0, 0, 1, 0};

    return sensor_msgs::CameraInfoPtr(camera_info_msg);
}

void PointcloudGenerator::updateObstacles(const envsim_msgs::ObstacleArrayConstPtr& obstacleArray) {
    this->obstacles = obstacleArray;
    this->obstacle_pointcloud_publisher.publish(this->constructPointcloudFromObstacles(this->obstacles));
}