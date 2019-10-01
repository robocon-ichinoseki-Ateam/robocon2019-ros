#include "../../util/util.h"

#include "OverlayText.h"

std_msgs::Float32 linear_data, angular_data;
float pose_arry[3] = {0};
float pre_pose[3] = {0};
int line_sensor[2] = {0};

float lift_height = 0.7;

void callbackAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
{
    pose_arry[0] = msgAMCL->pose.pose.position.x;
    pose_arry[1] = msgAMCL->pose.pose.position.y;
    pose_arry[2] = tf::getYaw(msgAMCL->pose.pose.orientation);
}

#include "FieldMarker.h"
#include "RobotMarker.h"

void callbackLineSensor_x(const std_msgs::Int32 &msg_line_sensor)
{
    line_sensor[0] = msg_line_sensor.data;
}

void callbackLineSensor_y(const std_msgs::Int32 &msg_line_sensor)
{
    line_sensor[1] = msg_line_sensor.data;
}

void callbackLiftHeight(const std_msgs::Float32 &msg_lift_height)
{
    lift_height = msg_lift_height.data;
}

int count = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    jsk_rviz_plugins::OverlayText text;

    ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, callbackAmcl);
    ros::Subscriber sub_line_sensor_x = nh.subscribe("line_sensor/binarized/x", 100, callbackLineSensor_x);
    ros::Subscriber sub_line_sensor_y = nh.subscribe("line_sensor/binarized/y", 100, callbackLineSensor_y);
    ros::Subscriber sub_lift_height = nh.subscribe("lift_height", 100, callbackLiftHeight);

    // テキスト
    ros::Publisher text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("display_rviz/text", 1);

    // フィールド
    ros::Publisher pub_field_mk = nh.advertise<visualization_msgs::MarkerArray>("field_marker", 1);

    // ロボット
    ros::Publisher pub_robot_footprint_mk = nh.advertise<visualization_msgs::Marker>("robot_marker/footprint", 1);
    ros::Publisher pub_line_sensor_mk_x = nh.advertise<visualization_msgs::MarkerArray>("robot_marker/line_sensor/x", 1);
    ros::Publisher pub_line_sensor_mk_y = nh.advertise<visualization_msgs::MarkerArray>("robot_marker/line_sensor/y", 1);
    ros::Publisher pub_robot_lift_mk = nh.advertise<visualization_msgs::Marker>("robot_marker/lift", 1);

    while (ros::ok())
    {
        // フィールドマーカーの描画は処理が重いため、最初の10秒のみ送信
        if (count < 10 * 30)
        {
            pub_field_mk.publish(generateDisplayField());
        }
        count++;

        // ロボットのマーカーを送信
        pub_robot_footprint_mk.publish(generateDisplayRobotFootprint());
        pub_line_sensor_mk_x.publish(generateDisplayLinesensor(0, line_sensor[0]));
        pub_line_sensor_mk_y.publish(generateDisplayLinesensor(1, line_sensor[1]));
        pub_robot_lift_mk.publish(generateDisplayRobotLift(lift_height));

        // 表示用文字列を生成し、ロボットの座標をrvizに表示
        std::string send_str = generateDisplayStr(pose_arry);
        configOverlayText(text, send_str);
        text_pub.publish(text);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
