#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double odom[3] = {0.0};
float pose_arry[3] = {0.0};

// mbed_to_ros のコールバック
void callbackFromMbed(const std_msgs::Float32MultiArray &msg)
{
    odom[0] = msg.data[1];
    odom[1] = msg.data[2];
    odom[2] = msg.data[3];
}

// robot_pose のコールバック
void callbackFromPose(const geometry_msgs::Pose2D &pose)
{
    pose_arry[0] = pose.x;
    pose_arry[1] = pose.y;
    pose_arry[2] = pose.theta;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_mbed_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_from_mbed = nh.subscribe("mbed_to_ros", 1000, callbackFromMbed);

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    current_time = ros::Time::now();

    ros::Publisher pub_to_mbed = nh.advertise<std_msgs::Float32MultiArray>("ros_to_mbed", 100);
    ros::Subscriber sub_pose = nh.subscribe("robot_pose", 1000, callbackFromPose);

    ros::Rate r(30.0);

    while (nh.ok())
    {
        current_time = ros::Time::now();

        //tf odom->base_link
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = odom[0];
        odom_trans.transform.translation.y = odom[1];
        odom_trans.transform.translation.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom[2]);
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // mbed へのデータ送信
        std_msgs::Float32MultiArray toMbed;
        toMbed.data.resize(4);
        toMbed.data[0] = 0.0;
        toMbed.data[1] = pose_arry[0];
        toMbed.data[2] = pose_arry[1];
        toMbed.data[3] = pose_arry[2];
        pub_to_mbed.publish(toMbed);

        ros::spinOnce();
        r.sleep();
    }
}