#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <icra_dodgedrone/odometry_transformer.h>
#include <icra_dodgedrone/pointcloud_generator.h>
#include <icra_dodgedrone/dodgedrone_map_utility.h>
#include <icra_dodgedrone/dodgedrone_command_wrapper.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "icra_dodgedrone_wrapper");
    ros::NodeHandle nh;

    ros::Publisher off = nh.advertise<std_msgs::Empty>("/kingfisher/dodgeros_pilot/off", 1);
    ros::Publisher reset = nh.advertise<std_msgs::Empty>("/kingfisher/dodgeros_pilot/reset_sim", 1);
    ros::Publisher enable = nh.advertise<std_msgs::Bool>("/kingfisher/dodgeros_pilot/enable", 1);
    ros::Publisher start = nh.advertise<std_msgs::Empty>("/kingfisher/dodgeros_pilot/start", 1);

    OdometryTransformer transformer(nh);
    PointcloudGenerator generator(nh);
    DodgeDroneMapUtility map_utility(nh);
    DodgeDroneCommandWrapper command_wrapper(nh);

    int count = 0;

    while(nh.ok()) {
        if(count == 0) {
            off.publish(std_msgs::EmptyConstPtr(new std_msgs::Empty));
            off.publish(std_msgs::EmptyConstPtr(new std_msgs::Empty));
            ros::Duration(0.25).sleep();
            reset.publish(std_msgs::EmptyConstPtr(new std_msgs::Empty));
            reset.publish(std_msgs::EmptyConstPtr(new std_msgs::Empty));
            ros::Duration(0.25).sleep();
            reset.publish(std_msgs::EmptyConstPtr(new std_msgs::Empty));

            std_msgs::Bool enable_msg;
            enable_msg.data = true;
            enable.publish(enable_msg);

            ros::Duration(0.25).sleep();
            count++;

            start.publish(std_msgs::EmptyConstPtr(new std_msgs::Empty));
        }

        ros::spinOnce();
    }
}