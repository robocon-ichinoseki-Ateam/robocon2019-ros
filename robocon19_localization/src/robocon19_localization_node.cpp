#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <string>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <math.h>

// void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
// {
//     float roll, pitch, yaw;
//     geometry_quat_to_rpy(roll, pitch, yaw, msgAMCL->pose.pose.orientation);

//     pose_arry[0] = msgAMCL->pose.pose.position.x;
//     pose_arry[1] = msgAMCL->pose.pose.position.y;
//     pose_arry[2] = (float)yaw;
// }

geometry_msgs::Quaternion toQuaternion(float roll, float pitch, float yaw)
{
    float cosRoll = cos(roll / 2.0);
    float sinRoll = sin(roll / 2.0);
    float cosPitch = cos(pitch / 2.0);
    float sinPitch = sin(pitch / 2.0);
    float cosYaw = cos(yaw / 2.0);
    float sinYaw = sin(yaw / 2.0);

    geometry_msgs::Quaternion q;
    q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;

    return q;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_localization_node");
    ros::NodeHandle nh;

    // ros::Subscriber sub_from_mbed = nh.subscribe("mbed_to_ros", 1000, callbackFromMbed);

    ros::Publisher pub_initialpose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);

    geometry_msgs::PoseWithCovarianceStamped initialpose;
    initialpose.header.frame_id = "map";
    initialpose.pose.pose.position.x = 0;
    initialpose.pose.pose.position.y = 0;
    initialpose.pose.pose.position.z = 0;
    initialpose.pose.pose.orientation = toQuaternion(0, 0, -45 * M_PI / 180);

    // ループ周波数を指定
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::Duration(10).sleep();
        ROS_INFO("!!");
        pub_initialpose.publish(initialpose);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
