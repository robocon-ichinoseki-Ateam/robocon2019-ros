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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_localization_node");
    ros::NodeHandle nh;

    // ループ周波数を指定
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
