#include <ros/ros.h>
#include <icra_dodgedrone/odometry_transformer.h>
#include <icra_dodgedrone/pointcloud_generator.h>
#include <icra_dodgedrone/dodgedrone_map_utility.h>
#include <icra_dodgedrone/dodgedrone_command_wrapper.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "icra_dodgedrone_wrapper");
    ros::NodeHandle nh;

    OdometryTransformer transformer(nh);
    PointcloudGenerator generator(nh);
    DodgeDroneMapUtility map_utility(nh);
    DodgeDroneCommandWrapper command_wrapper(nh);

    while(nh.ok()) {
        ros::spinOnce();
    }
}