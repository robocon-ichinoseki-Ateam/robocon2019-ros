#include "../../util/util.h"

void configOverlayText(jsk_rviz_plugins::OverlayText &t, std::string str);
std::string generateDisplayStr(float pose[3]);

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

visualization_msgs::MarkerArray generateDisplayField()
{
    visualization_msgs::MarkerArray m_a;
    m_a.markers.resize(15);

    for (int i = 0; i < 6; i++)
    {
        m_a.markers[i].header.frame_id = "/map";
        m_a.markers[i].header.stamp = ros::Time::now();
        m_a.markers[i].ns = "robocon19_node";
        m_a.markers[i].id = 50 + i;
        m_a.markers[i].lifetime = ros::Duration();

        m_a.markers[i].type = visualization_msgs::Marker::CUBE;
        m_a.markers[i].action = visualization_msgs::Marker::ADD;

        m_a.markers[i].scale.x = 0.35;
        m_a.markers[i].scale.y = 0.40;
        m_a.markers[i].scale.z = 0.30;

        m_a.markers[i].pose.position.z = 0.15;
        m_a.markers[i].pose.orientation.x = 0;
        m_a.markers[i].pose.orientation.y = 0;
        m_a.markers[i].pose.orientation.z = 0;
        m_a.markers[i].pose.orientation.w = 1;

        m_a.markers[i].color.r = 1.0f;
        m_a.markers[i].color.g = 0.0f;
        m_a.markers[i].color.b = 1.0f;
        m_a.markers[i].color.a = 1.0f;
    }

    m_a.markers[0].pose.position.x = -0.825;
    m_a.markers[0].pose.position.y = 6.7;
    m_a.markers[1].pose.position.x = -0.825;
    m_a.markers[1].pose.position.y = 6.7 - 2;
    m_a.markers[2].pose.position.x = -0.825;
    m_a.markers[2].pose.position.y = 6.7 - 4;
    m_a.markers[3].pose.position.x = -0.975 - 3.15;
    m_a.markers[3].pose.position.y = 6.7;
    m_a.markers[4].pose.position.x = -0.975 - 3.15;
    m_a.markers[4].pose.position.y = 6.7 - 2;
    m_a.markers[5].pose.position.x = -0.975 - 3.15;
    m_a.markers[5].pose.position.y = 6.7 - 4;

    for (int i = 6; i < 12; i++)
    {
        m_a.markers[i].header.frame_id = "/map";
        m_a.markers[i].header.stamp = ros::Time::now();
        m_a.markers[i].ns = "robocon19_node";
        m_a.markers[i].id = 50 + i;
        m_a.markers[i].lifetime = ros::Duration();

        m_a.markers[i].type = visualization_msgs::Marker::CYLINDER;
        m_a.markers[i].action = visualization_msgs::Marker::ADD;

        m_a.markers[i].scale.x = 0.065;
        m_a.markers[i].scale.y = 0.065;

        m_a.markers[i].pose.position.z = 0.15;
        m_a.markers[i].pose.orientation.x = 0;
        m_a.markers[i].pose.orientation.y = 0;
        m_a.markers[i].pose.orientation.z = 0;
        m_a.markers[i].pose.orientation.w = 1;

        m_a.markers[i].color.r = 1.0f;
        m_a.markers[i].color.g = 0.0f;
        m_a.markers[i].color.b = 1.0f;
        m_a.markers[i].color.a = 1.0f;
    }

    m_a.markers[6].scale.z = 2;
    m_a.markers[6].pose.position.x = -0.825 - 0.125;
    m_a.markers[6].pose.position.y = 6.7;
    m_a.markers[6].pose.position.z = 1.0;
    m_a.markers[7].scale.z = 1.5;
    m_a.markers[7].pose.position.x = -0.825 - 0.125;
    m_a.markers[7].pose.position.y = 6.7 - 2;
    m_a.markers[7].pose.position.z = 0.75;
    m_a.markers[8].scale.z = 1;
    m_a.markers[8].pose.position.x = -0.825 - 0.125;
    m_a.markers[8].pose.position.y = 6.7 - 4;
    m_a.markers[8].pose.position.z = 0.5;
    m_a.markers[9].scale.z = 2;
    m_a.markers[9].pose.position.x = -0.975 - 3.15 + 0.125;
    m_a.markers[9].pose.position.y = 6.7;
    m_a.markers[9].pose.position.z = 1.0;
    m_a.markers[10].scale.z = 1.5;
    m_a.markers[10].pose.position.x = -0.975 - 3.15 + 0.125;
    m_a.markers[10].pose.position.y = 6.7 - 2;
    m_a.markers[10].pose.position.z = 0.75;
    m_a.markers[11].scale.z = 1;
    m_a.markers[11].pose.position.x = -0.975 - 3.15 + 0.125;
    m_a.markers[11].pose.position.y = 6.7 - 4;
    m_a.markers[11].pose.position.z = 0.5;

    for (int i = 12; i < 15; i++)
    {
        m_a.markers[i].header.frame_id = "/map";
        m_a.markers[i].header.stamp = ros::Time::now();
        m_a.markers[i].ns = "robocon19_node";
        m_a.markers[i].id = 50 + i;
        m_a.markers[i].lifetime = ros::Duration();

        m_a.markers[i].type = visualization_msgs::Marker::CYLINDER;
        m_a.markers[i].action = visualization_msgs::Marker::ADD;

        m_a.markers[i].scale.x = 0.065;
        m_a.markers[i].scale.y = 0.065;

        m_a.markers[i].pose.position.z = 0.15;
        m_a.markers[i].pose.orientation.x = -0.5;
        m_a.markers[i].pose.orientation.y = 0.5;
        m_a.markers[i].pose.orientation.z = -0.5;
        m_a.markers[i].pose.orientation.w = 0.5;

        m_a.markers[i].color.r = 1.0f;
        m_a.markers[i].color.g = 0.0f;
        m_a.markers[i].color.b = 1.0f;
        m_a.markers[i].color.a = 1.0f;
    }

    m_a.markers[12].scale.z = 3.0;
    m_a.markers[12].pose.position.x = -2.475;
    m_a.markers[12].pose.position.y = 6.7;
    m_a.markers[12].pose.position.z = 2.0;
    m_a.markers[13].scale.z = 3.0;
    m_a.markers[13].pose.position.x = -2.475;
    m_a.markers[13].pose.position.y = 6.7 - 2;
    m_a.markers[13].pose.position.z = 1.5;
    m_a.markers[14].scale.z = 3.0;
    m_a.markers[14].pose.position.x = -2.475;
    m_a.markers[14].pose.position.y = 6.7 - 4;
    m_a.markers[14].pose.position.z = 1.0;

    return m_a;
}

visualization_msgs::MarkerArray generateDisplayLinesensor(int sensor_id, float attach_pose[2], int snesor_val)
{
    visualization_msgs::MarkerArray m_a;

    const float interval = 0.01397;
    const int s_num = 7;

    m_a.markers.resize(s_num);

    for (int i = 0; i < s_num; i++)
    {
        m_a.markers[i].header.frame_id = "/base_link";
        m_a.markers[i].header.stamp = ros::Time::now();
        m_a.markers[i].ns = "robocon19_node";
        m_a.markers[i].id = i;
        m_a.markers[i].lifetime = ros::Duration();

        m_a.markers[i].type = visualization_msgs::Marker::CUBE;
        m_a.markers[i].action = visualization_msgs::Marker::ADD;
        m_a.markers[i].scale.x = interval;
        m_a.markers[i].scale.y = interval;
        m_a.markers[i].scale.z = 0.001;

        if (sensor_id == 0)
        {
            m_a.markers[i].pose.position.x = attach_pose[0] + (float)i * interval - interval * 3.5;
            m_a.markers[i].pose.position.y = attach_pose[1];
        }
        else
        {
            m_a.markers[i].pose.position.x = attach_pose[0];
            m_a.markers[i].pose.position.y = attach_pose[1] + (float)i * interval - interval * 3.5;
        }

        m_a.markers[i].pose.position.z = 0;
        m_a.markers[i].pose.orientation.x = 0;
        m_a.markers[i].pose.orientation.y = 0;
        m_a.markers[i].pose.orientation.z = 0;
        m_a.markers[i].pose.orientation.w = 1;

        if ((snesor_val >> i) & 1)
        {
            m_a.markers[i].color.r = 1.0f;
            m_a.markers[i].color.g = 0.0f;
            m_a.markers[i].color.b = 1.0f;
        }
        else
        {
            m_a.markers[i].color.r = 1.0f;
            m_a.markers[i].color.g = 1.0f;
            m_a.markers[i].color.b = 0.0f;
        }

        m_a.markers[i].color.a = 1.0f;
    }

    return m_a;
}

float f = 0.0;
visualization_msgs::Marker generateDisplayRobot()
{
    visualization_msgs::Marker line_strip;
    line_strip.id = 100;
    line_strip.header.frame_id = "/base_link";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "robocon19_node";
    line_strip.lifetime = ros::Duration();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.02;

    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    float footprint_point_f[13][2] = {{0.3575, 0.279725}, {0.279725, 0.3575}, {-0.279725, 0.3575}, {-0.3575, 0.279725}, {-0.3575, -0.279725}, {-0.279725, -0.3575}, {-0.046, -0.3575}, {-0.031, -0.4275}, {0.031, -0.4275}, {0.046, -0.3575}, {0.279725, -0.3575}, {0.3575, -0.279725}, {0.3575, 0.279725}};

    for (uint32_t i = 0; i < 13; i++)
    {
        geometry_msgs::Point p;
        p.x = footprint_point_f[i][0];
        p.y = footprint_point_f[i][1];
        p.z = 0;

        line_strip.points.push_back(p);
    }

    return line_strip;
}

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

    ros::Publisher pub_robot_mk = nh.advertise<visualization_msgs::Marker>("robot_marker", 1);

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

        pub_robot_mk.publish(generateDisplayRobot());

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
