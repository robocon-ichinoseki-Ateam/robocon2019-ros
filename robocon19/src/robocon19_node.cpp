#include "../../util/util.h"

void configOverlayText(jsk_rviz_plugins::OverlayText &t, std::string str);
std::string generateDisplayStr(float pose[3]);

std_msgs::Float32 linear_data, angular_data;

float pose_arry[3] = {0};
float pre_pose[3] = {0};

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
{
    static ros::Time pre_time;
    ros::Duration ros_duration = ros::Time::now() - pre_time;
    float dt = (float)ros_duration.sec + (float)ros_duration.nsec * pow(10, -9);
    pre_time = ros::Time::now();

    double roll, pitch, yaw;
    geometry_quat_to_rpy(roll, pitch, yaw, msgAMCL->pose.pose.orientation);

    pose_arry[0] = msgAMCL->pose.pose.position.x;
    pose_arry[1] = msgAMCL->pose.pose.position.y;
    pose_arry[2] = (float)yaw;

    float dx = (pose_arry[0] - pre_pose[0]);
    float dy = (pose_arry[1] - pre_pose[1]);
    linear_data.data = sqrt(dx * dx + dy * dy) / dt;
    angular_data.data = (pose_arry[2] - pre_pose[2]) / dt;

    for (int i = 0; i < 3; i++)
        pre_pose[i] = pose_arry[i];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_node");
    ros::NodeHandle nh;

    jsk_rviz_plugins::OverlayText text;
    ros::Publisher text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("text", 1);

    ros::Publisher linear_pub = nh.advertise<std_msgs::Float32>("linear_v", 10);
    ros::Publisher angular_pub = nh.advertise<std_msgs::Float32>("angular_v", 10);

    // ros::Subscriber sub_pose = nh.subscribe("robot_pose", 1000, callbackFromPose);
    ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, poseAMCLCallback);

    // ループ周波数を指定
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        linear_pub.publish(linear_data);
        angular_pub.publish(angular_data);

        // 表示用文字列を生成し、ロボットの座標をrvizに表示
        std::string send_str = generateDisplayStr(pose_arry);
        configOverlayText(text, send_str);
        text_pub.publish(text);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
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
