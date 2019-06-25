#include "../../util/util.h"

float odom_arry[3] = {0};
int line_arry[2] = {0};

geometry_msgs::Twist cmd_vel;
void joy_callback(const sensor_msgs::Joy &joy_msg)
{
    cmd_vel.linear.x = -joy_msg.axes[0];
    cmd_vel.linear.y = joy_msg.axes[1];
    cmd_vel.angular.z = 1.5 * joy_msg.axes[3];
}

void callbackOdom(const nav_msgs::Odometry &msg)
{
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, msg.pose.pose.orientation);

    odom_arry[0] = msg.pose.pose.position.x;
    odom_arry[1] = msg.pose.pose.position.y;
    odom_arry[2] = tf::getYaw(msg.pose.pose.orientation);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_sim_stage");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    //publish
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Publisher pub_to_ros = nh.advertise<std_msgs::Float32MultiArray>("mbed_to_ros", 100);

    //subscriibe
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);
    ros::Subscriber sub_odom = nh.subscribe("odom", 10, callbackOdom);

    while (ros::ok())
    {
        cmd_pub.publish(cmd_vel);

        // mbed へのデータ送信
        std_msgs::Float32MultiArray toRos;
        toRos.data.resize(6);
        toRos.data[0] = 0.0;
        toRos.data[1] = odom_arry[0];
        toRos.data[2] = odom_arry[1];
        toRos.data[3] = odom_arry[2];
        toRos.data[4] = line_arry[0];
        toRos.data[5] = line_arry[1];
        pub_to_ros.publish(toRos);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}