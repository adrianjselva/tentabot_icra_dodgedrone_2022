#include "icra_dodgedrone/pointcloud_generator.h"

PointcloudGenerator::PointcloudGenerator(ros::NodeHandle &n) : nh_(n), it_(n) {
    this->depth_subscriber = it_.subscribe("/kingfisher/dodgeros_pilot/unity/depth",
                                           1,
                                           &PointcloudGenerator::transformDepth,
                                           this);

    this->camera_info_publisher = nh_.advertise<sensor_msgs::CameraInfo>("/kingfisher/camera_info", 1);
    this->pointcloud_publisher = nh_.advertise<sensor_msgs::PointCloud2>("/kingfisher/pointcloud", 1);
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
    this->pointcloud_publisher.publish(this->constructPointcloudFromDepth(depth, camera_info));
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

sensor_msgs::CameraInfoPtr PointcloudGenerator::simulatedCameraInfo() {
    auto *camera_info_msg = new sensor_msgs::CameraInfo;
    camera_info_msg->header.frame_id = "camera";
    camera_info_msg->width = 320;
    camera_info_msg->height = 240;
    camera_info_msg->K = {142.82927, 0, 160.0, 0, 142.82927, 120.0, 0, 0, 1};
    camera_info_msg->P = {142.82927, 0, 160.0, 0, 0, 142.82927, 120.0, 0, 0, 0, 1, 0};

    return sensor_msgs::CameraInfoPtr(camera_info_msg);
}