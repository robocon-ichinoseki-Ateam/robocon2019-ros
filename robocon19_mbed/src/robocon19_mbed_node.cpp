#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
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

    // static ros::Time pre_time;
    // ros::Duration ros_duration = ros::Time::now() - pre_time;
    // float dt = (float)ros_duration.sec + (float)ros_duration.nsec * pow(10, -9);
    // pre_time = ros::Time::now();
    // ROS_INFO("dt: %f[sec]\tf: %f[Hz]", dt, 1 / dt);
}

// クウォータニオンからオイラー角を返す
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Pass by Reference
}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
{
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, msgAMCL->pose.pose.orientation);

    pose_arry[0] = msgAMCL->pose.pose.position.x;
    pose_arry[1] = msgAMCL->pose.pose.position.y;
    pose_arry[2] = (float)yaw;
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
    ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, poseAMCLCallback);

    ros::Rate loop_rate(50.0);

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
        loop_rate.sleep();
    }
}