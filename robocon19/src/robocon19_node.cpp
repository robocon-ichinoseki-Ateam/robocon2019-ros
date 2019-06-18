#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include "geometry_msgs/Pose2D.h"
#include <string>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <math.h>

void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat);
void getRobotPose(float return_pose[3]);
void configOverlayText(jsk_rviz_plugins::OverlayText &t, std::string str);
std::string generateDisplayStr(float pose[3]);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_node");
    ros::NodeHandle nh;

    jsk_rviz_plugins::OverlayText text;
    ros::Publisher text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("text", 1);

    ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose2D>("robot_pose", 10);
    geometry_msgs::Pose2D pose;

    // ループ周波数を指定
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        // ロボットの座標を取得
        float pose_arry[3] = {0};
        getRobotPose(pose_arry);

        // ロボットの座標をTopicとしてPublish
        pose.x = pose_arry[0];
        pose.y = pose_arry[1];
        pose.theta = pose_arry[2];
        pub_pose.publish(pose);

        // 表示用文字列を生成し、ロボットの座標をrvizに表示
        std::string send_str = generateDisplayStr(pose_arry);
        configOverlayText(text, send_str);
        text_pub.publish(text);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

// クウォータニオンからオイラー角を返す
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Pass by Reference
}

// TFを読んでロボットの座標と角度を返す
void getRobotPose(float return_pose[3])
{
    tf::TransformListener ln;

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
    }
    catch (...)
    {
        ROS_INFO("tf error");
    }

    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, target_pose.pose.orientation);

    return_pose[0] = target_pose.pose.position.x;
    return_pose[1] = target_pose.pose.position.y;
    return_pose[2] = yaw;
}

// rvizに表示するテキストの設定
void configOverlayText(jsk_rviz_plugins::OverlayText &t, std::string str)
{
    t.action = jsk_rviz_plugins::OverlayText::ADD;
    t.width = 140;
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

// rvizに表示するテキストデータを生成
std::string generateDisplayStr(float pose[3])
{
    std::stringstream ss;
    ss << "-position-\r\n"
       << "x: " << pose[0] << "\r\n"
       << "y: " << pose[1] << "\r\n"
       << "z: " << pose[2] * (180 / 3.14159); /*to degree*/

    return ss.str();
}
