#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double state_odom_x = 0.0;  //オドメトリX座標[m]
double state_odom_y = 0.0;  //オドメトリY座標[m]
double state_odom_th = 0.0; //オドメトリ姿勢[rad]

void chatterCallback(const std_msgs::Float32MultiArray &msg)
{
    int num = msg.data.size();
    state_odom_x = msg.data[1];
    state_odom_y = msg.data[2];
    state_odom_th = msg.data[3];

    ROS_INFO("I susclibed [%i]", num);
    for (int i = 0; i < num; i++)
    {
        ROS_INFO("[%i]:%f", i, msg.data[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_mbed_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("mbed_to_ros", 1000, chatterCallback);

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    current_time = ros::Time::now();

    ros::Rate r(30.0);

    while (nh.ok())
    {
        current_time = ros::Time::now();

        //tf odom->base_link
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = state_odom_x;
        odom_trans.transform.translation.y = state_odom_y;
        odom_trans.transform.translation.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state_odom_th);
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        ros::spinOnce();
        r.sleep();
    }
}