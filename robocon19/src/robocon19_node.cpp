#include "../../util/util.h"

#include "OverlayText.h"

std_msgs::Float32 linear_data, angular_data;
float pose_arry[3] = {0};
float pre_pose[3] = {0};
int line_sensor[2] = {0};

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

int count = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    jsk_rviz_plugins::OverlayText text;

    ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, callbackAmcl);
    ros::Subscriber sub_line_sensor_x = nh.subscribe("line_sensor/binarized/x", 100, callbackLineSensor_x);
    ros::Subscriber sub_line_sensor_y = nh.subscribe("line_sensor/binarized/y", 100, callbackLineSensor_y);

    // 表示系
    ros::Publisher text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("display_rviz/text", 1);
    ros::Publisher pub_line_sensor_mk_x = nh.advertise<visualization_msgs::MarkerArray>("line_sensor_marker/x", 1);
    ros::Publisher pub_line_sensor_mk_y = nh.advertise<visualization_msgs::MarkerArray>("line_sensor_marker/y", 1);

    ros::Publisher pub_field_mk = nh.advertise<visualization_msgs::MarkerArray>("field_marker", 1);

    ros::Publisher pub_robot_footprint_mk = nh.advertise<visualization_msgs::Marker>("robot_marker/footprint", 1);
    ros::Publisher pub_robot_lift_mk = nh.advertise<visualization_msgs::Marker>("robot_marker/lift", 1);

    while (ros::ok())
    {
        float attach_pose[2][2] = {{0, -0.295}, {-0.295, 0}};
        pub_line_sensor_mk_x.publish(generateDisplayLinesensor(0, attach_pose[0], line_sensor[0]));
        pub_line_sensor_mk_y.publish(generateDisplayLinesensor(1, attach_pose[1], line_sensor[1]));

        // 処理が重いため、最初の10秒のみ送信
        if (count < 10000 * 50)
        {
            pub_field_mk.publish(generateDisplayField());
        }
        count++;

        pub_robot_footprint_mk.publish(generateDisplayRobotFootprint());
        pub_robot_lift_mk.publish(generateDisplayRobotLift());

        // 表示用文字列を生成し、ロボットの座標をrvizに表示
        std::string send_str = generateDisplayStr(pose_arry);
        configOverlayText(text, send_str);
        text_pub.publish(text);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
