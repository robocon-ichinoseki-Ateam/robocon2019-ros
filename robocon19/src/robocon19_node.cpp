#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <string>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <sstream>

#include <string>
#include <math.h>

void sendText(jsk_rviz_plugins::OverlayText &t, std::string str)
{
    t.action = jsk_rviz_plugins::OverlayText::ADD;
    t.width = 100;
    t.height = 100;
    t.left = 10;
    t.top = 10;

    std_msgs::ColorRGBA color1, color2;
    color1.r = 0;
    color1.g = 0;
    color1.b = 0;
    color1.a = 0.4;
    t.bg_color = color1;

    color2.r = 25.0 / 255;
    color2.g = 255.0 / 255;
    color2.b = 240.0 / 255;
    color2.a = 0.8;
    t.fg_color = color2;

    t.line_width = 1;
    t.text_size = 14;
    t.font = "Ubuntu";
    t.text = str;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_node");
    ros::NodeHandle nh;

    tf::TransformListener ln;

    jsk_rviz_plugins::OverlayText text;
    ros::Publisher text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("text", 1);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        geometry_msgs::PoseStamped source_pose;
        source_pose.header.frame_id = "base_link";
        source_pose.header.stamp = ros::Time(0);
        source_pose.pose.orientation.w = 1.0;

        geometry_msgs::PoseStamped target_pose;
        std::string target_frame = "map";
        try
        {
            ln.waitForTransform(source_pose.header.frame_id, target_frame, ros::Time(0), ros::Duration(1.0));
            ln.transformPose(target_frame, source_pose, target_pose);

            std::stringstream ss;
            ss << "-position-\r\n"
               << "x: " << target_pose.pose.position.x << "\r\n"
               << "y: " << target_pose.pose.position.y;

            std::string send_str = ss.str();

            sendText(text, send_str);
            text_pub.publish(text);

            ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        }
        catch (...)
        {
            ROS_INFO("tf error");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}