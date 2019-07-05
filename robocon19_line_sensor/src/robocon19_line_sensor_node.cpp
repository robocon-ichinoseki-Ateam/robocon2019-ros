#include "../../util/util.h"

int binarized_data[2] = {0};

enum
{
    red = 3,
    green,
    blue
};

int use_color = blue;

int line_sensor_th[7] = {
    0.6,
    0.6,
    0.6,
    0.6,
    0.6,
    0.6,
    0.6,
};

// センサのデータを2値化
int binarizeLinseSensor(float data[9])
{
    int binarized = 0;

    // 0 to 2
    for (int i = 0; i <= 2; i++)
    {
        int on_line = (data[i] < 0.6) ? 1 : 0;
        binarized += on_line << i;
    }

    // 3 中央のセンサは3色のうち一つのみを使用する
    int on_line = (data[use_color] < 0.6) ? 1 : 0;
    binarized += on_line << 3;

    // 4 to 6
    for (int i = 4; i <= 6; i++)
    {
        int on_line = (data[i + 2] < 0.6) ? 1 : 0;
        binarized += on_line << i;
    }

    return binarized;
}

void callbackLineSensor_x(const std_msgs::Float32MultiArray &msg_line_sensor)
{
    // データを配列にコピー
    float data[9];
    for (int i = 0; i < 9; i++)
        data[i] = msg_line_sensor.data[i];

    binarized_data[0] = binarizeLinseSensor(data);
}

void callbackLineSensor_y(const std_msgs::Float32MultiArray &msg_line_sensor)
{
    // データを配列にコピー
    float data[9];
    for (int i = 0; i < 9; i++)
        data[i] = msg_line_sensor.data[i];
    data[8] = 1;

    binarized_data[1] = binarizeLinseSensor(data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_line_sensor_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Subscriber sub_line_sensor_x = nh.subscribe("line_sensor/x", 100, callbackLineSensor_x);
    ros::Subscriber sub_line_sensor_y = nh.subscribe("line_sensor/y", 100, callbackLineSensor_y);

    ros::Publisher pub_line_sensor_x = nh.advertise<std_msgs::Int32>("line_sensor/binarized/x", 1);
    ros::Publisher pub_line_sensor_y = nh.advertise<std_msgs::Int32>("line_sensor/binarized/y", 1);

    while (ros::ok())
    {
        std_msgs::Int32 binarized_x, binarized_y;
        binarized_x.data = binarized_data[0];
        binarized_y.data = binarized_data[1];
        pub_line_sensor_x.publish(binarized_x);
        pub_line_sensor_y.publish(binarized_y);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
