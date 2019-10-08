#include "../../util/util.h"

float pose_arry[3] = {0.0};
int line_sensor_arry[2] = {0};
float odom_arry[3] = {0.0};

geometry_msgs::TransformStamped odom;
std_msgs::Float32 lift_height;
std_msgs::Bool needs_reset_pose;
bool init_flag = false;

// Mbedからのコールバック
void callbackFromMbed(const std_msgs::Float32MultiArray &msg)
{
    needs_reset_pose.data = (bool)msg.data[0];
    odom.transform.translation.x = msg.data[1];
    odom.transform.translation.y = msg.data[2];
    odom.transform.rotation = tf::createQuaternionMsgFromYaw(msg.data[3]);
    lift_height.data = msg.data[4];
    line_sensor_arry[1] = msg.data[5];

    for (int i = 0; i < 3; i++)
        odom_arry[i] = msg.data[i + 1];

    if (!init_flag)
    {
        if (abs(msg.data[1]) < 0.001 && 0.242 < msg.data[2] && msg.data[2] < 0.244)
        {
            needs_reset_pose.data = true;
            init_flag = true;
        }
    }
    // measureLooptime();
}

// amclからのコールバック
void callbackAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg_amcl)
{
    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, msg_amcl->pose.pose.orientation);

    pose_arry[0] = msg_amcl->pose.pose.position.x;
    pose_arry[1] = msg_amcl->pose.pose.position.y;
    pose_arry[2] = (float)yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_mbed_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50.0);

    // mbedからのデータ受信者
    ros::Subscriber sub_from_mbed = nh.subscribe("mbed_to_ros", 1000, callbackFromMbed);

    // amclからのデータ受信者
    ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, callbackAmcl);

    // オドメトリの配信者
    tf::TransformBroadcaster odom_broadcaster;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Mbedへのデータ配信者
    ros::Publisher pub_odom_data = nh.advertise<std_msgs::Float32MultiArray>("odom_data", 100);

    // odomデータ配信者
    ros::Publisher pub_to_mbed = nh.advertise<std_msgs::Float32MultiArray>("ros_to_mbed", 100);

    // 昇降機構の高さのデータ配信者
    ros::Publisher pub_lift_height = nh.advertise<std_msgs::Float32>("lift_height", 100);

    // 自己位置リセット信号の配信者
    ros::Publisher pub_reset = nh.advertise<std_msgs::Bool>("reset", 100);
    while (nh.ok())
    {
        // ROS_INFO("%d", needs_reset_pose.data);

        odom.header.stamp = ros::Time::now();
        odom_broadcaster.sendTransform(odom);

        // mbed へのデータ送信
        std_msgs::Float32MultiArray toMbed;
        toMbed.data.resize(4);
        toMbed.data[0] = 0.0;
        toMbed.data[1] = pose_arry[0];
        toMbed.data[2] = pose_arry[1];
        toMbed.data[3] = pose_arry[2];
        pub_to_mbed.publish(toMbed);

        // odom_data送信
        std_msgs::Float32MultiArray odom_data;
        odom_data.data.resize(3);
        odom_data.data[0] = odom_arry[0];
        odom_data.data[1] = odom_arry[1];
        odom_data.data[2] = odom_arry[2];
        pub_odom_data.publish(odom_data);

        // 昇降機構の高さの送信
        pub_lift_height.publish(lift_height);

        // リセット信号の送信
        pub_reset.publish(needs_reset_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }
}