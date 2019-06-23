#include "../../util/util.h"

bool needs_reset = false;

void callbackReset(const std_msgs::Bool &msg_reset)
{
    needs_reset = msg_reset.data | needs_reset;
}

// 初期化位置の生成
geometry_msgs::PoseWithCovarianceStamped
generateInitialpose(float x, float y, float yaw)
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0;
    pose.pose.pose.orientation = toQuaternion(0, 0, yaw);

    return pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_localization_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    // amclからのデータ受信者
    ros::Subscriber sub_amcl = nh.subscribe("reset", 100, callbackReset);

    // 位置初期化データの送信者
    ros::Publisher pub_initialpose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);

    while (ros::ok())
    {
        if (needs_reset)
        {
            pub_initialpose.publish(generateInitialpose(0, 0, 0));
            needs_reset = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
