#include "icra_dodgedrone/odometry_transformer.h"

void OdometryTransformer::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped map;

    map.header.stamp = ros::Time::now();
    map.header.frame_id = "world";
    map.child_frame_id = "map";
    map.transform.translation.x = msg->pose.pose.position.x;
    map.transform.translation.y = msg->pose.pose.position.y;
    map.transform.translation.z = msg->pose.pose.position.z;
    map.transform.rotation.x = msg->pose.pose.orientation.x;
    map.transform.rotation.y = msg->pose.pose.orientation.y;
    map.transform.rotation.z = msg->pose.pose.orientation.z;
    map.transform.rotation.w = msg->pose.pose.orientation.w;

    br.sendTransform(map);

    geometry_msgs::TransformStamped kingfisher;

    kingfisher.header.stamp = ros::Time::now();
    kingfisher.header.frame_id = "map";
    kingfisher.child_frame_id = "kingfisher";
    kingfisher.transform.translation.x = 0;
    kingfisher.transform.translation.y = 0;
    kingfisher.transform.translation.z = 0;
    kingfisher.transform.rotation.x = msg->pose.pose.orientation.x;
    kingfisher.transform.rotation.y = msg->pose.pose.orientation.y;
    kingfisher.transform.rotation.z = msg->pose.pose.orientation.z;
    kingfisher.transform.rotation.w = msg->pose.pose.orientation.w;

    br.sendTransform(kingfisher);

    geometry_msgs::TransformStamped camera;

    camera.header.stamp = ros::Time::now();
    camera.header.frame_id = "kingfisher";
    camera.child_frame_id = "camera";
    camera.transform.translation.x = 0;
    camera.transform.translation.y = 0;
    camera.transform.translation.z = 0.3;

    tf2::Quaternion q;
    q.setRPY(-M_PI_2, 0, -M_PI_2);
    camera.transform.rotation.x = q.getX();
    camera.transform.rotation.y = q.getY();
    camera.transform.rotation.z = q.getZ();
    camera.transform.rotation.w = q.getW();

    br.sendTransform(camera);
}

OdometryTransformer::OdometryTransformer(ros::NodeHandle &n) : nh_(n){
    odometry_subscriber = nh_.subscribe("/kingfisher/dodgeros_pilot/odometry",
                                        1,
                                        &OdometryTransformer::odometryCallback,
                                        this);
}