#include "../../util/util.h"

int line_sensor[2] = {0};

void callbackLineSensor_x(const std_msgs::Float32MultiArray &msg_line_sensor)
{
    line_sensor[0] = 0;
    for (int i = 0; i < 7; i++)
    {
        int on_line = (msg_line_sensor.data[i] < 0.6) ? 1 : 0;
        line_sensor[0] += on_line << i;
    }
}

void callbackLineSensor_y(const std_msgs::Float32MultiArray &msg_line_sensor)
{
    line_sensor[1] = 0;
    for (int i = 0; i < 7; i++)
    {
        int on_line = (msg_line_sensor.data[i] < 0.6) ? 1 : 0;
        line_sensor[1] += on_line << i;
    }
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
        binarized_x.data = line_sensor[0];
        binarized_y.data = line_sensor[1];
        pub_line_sensor_x.publish(binarized_x);
        pub_line_sensor_y.publish(binarized_y);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
