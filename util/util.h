#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include <jsk_rviz_plugins/OverlayText.h>
#include <visualization_msgs/MarkerArray.h>

// クウォータニオンからオイラー角を返す
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Pass by Reference
}

// オイラー角からクウォータニオンを返す
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

// ループタイムの測定
// この関数が前回呼びだされてから何秒かかったかを表示
void measureLooptime()
{
    static ros::Time pre_time;
    ros::Duration ros_duration = ros::Time::now() - pre_time;
    float dt = (float)ros_duration.sec + (float)ros_duration.nsec * pow(10, -9);
    pre_time = ros::Time::now();
    ROS_INFO("dt: %f[sec]\tf: %f[Hz]", dt, 1 / dt);
}
