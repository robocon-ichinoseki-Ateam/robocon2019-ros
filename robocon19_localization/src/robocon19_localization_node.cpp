#include "../../util/util.h"

bool needs_reset = false;
float odom_data[3] = {0};

int pre_binarized_line[2] = {0};
int binarized_line[2] = {0};
float pose_arry[3] = {0};

// ^Crobo-ichinoseki-a@robo-ichinoseki-a:~/catkin_ws$ rostopic echo /initialpose
// header:
//   seq: 4
//   stamp:
//     secs: 1562306115
//     nsecs:   3863220
//   frame_id: "map"
// pose:
//   pose:
//     position:
//       x: -0.333866745234
//       y: 0.0744142234325
//       z: 0.0
//     orientation:
//       x: 0.0
//       y: 0.0
//       z: 0.0
//       w: 1.0
//   covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

void callbackOdomData(const std_msgs::Float32MultiArray &msg)
{
    for (int i = 0; i < 3; i++)
        odom_data[i] = msg.data[i];
}

void callbackAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
{
    pose_arry[0] = msgAMCL->pose.pose.position.x;
    pose_arry[1] = msgAMCL->pose.pose.position.y;
    pose_arry[2] = tf::getYaw(msgAMCL->pose.pose.orientation);
}

void callbackReset(const std_msgs::Bool &msg_reset)
{
    needs_reset = msg_reset.data | needs_reset;
}

void callbackLineX(const std_msgs::Int32 &line)
{
    binarized_line[0] = line.data;
}

void callbackLineY(const std_msgs::Int32 &line)
{
    binarized_line[1] = line.data;
}

// センサ間距離
float dis_between_sensor = 0.01397;

float sensor_pose[2];
void isOnLine(float x, float y, float arg, int line[2])
{
    line[0] = 0;
    line[1] = 0;

    for (int i = 0; i < 7; i++)
    {
        // /mapでのセンサの位置
        // float sensor_pose[2];

        // 機体中心からセンサまでの角度
        float sensor_arg = atan2((double)(i * dis_between_sensor - 3 * dis_between_sensor), pose_line_sensor);

        // 機体中心からセンサまでの距離
        float len = pose_line_sensor / cos(sensor_arg);

        sensor_pose[0] = x + len * cos(arg + sensor_arg - M_PI / 2);
        sensor_pose[1] = y + len * sin(arg + sensor_arg - M_PI / 2);

        if (((sensor_pose[0] < 0.025) && (sensor_pose[0] > -0.025)))
        {
            line[0] |= 1 << i;
        }
    }

    for (int i = 0; i < 7; i++)
    {
        // /mapでのセンサの位置
        // float sensor_pose[2];

        // 機体中心からセンサまでの角度
        float sensor_arg = atan2((double)(i * dis_between_sensor - 3 * dis_between_sensor), pose_line_sensor);

        // 機体中心からセンサまでの距離
        float len = pose_line_sensor / cos(sensor_arg);

        sensor_pose[0] = x + len * cos(arg + sensor_arg);
        sensor_pose[1] = y + len * sin(arg + sensor_arg);

        if (((sensor_pose[1] < 0.025) && (sensor_pose[1] > -0.025)))
        {
            line[1] |= 1 << i;
        }
    }
}

float trueth_pose[3];
void callbackparticlecloud(const geometry_msgs::PoseArray &particle)
{
    int size = particle.poses.size();

    if (!(binarized_line[0] || binarized_line[1]))
    {
        return;
    }

    // ROS_INFO("%d\t%f\r\n", size, particle.poses[0].position.x);
    int line_num = 0;
    float sum_line[3] = {0};

    for (int i = 0; i < 500; i++)
    {
        float x = particle.poses[i].position.x;
        float y = particle.poses[i].position.y;
        float arg = tf::getYaw(particle.poses[i].orientation);

        int line[2] = {0};
        isOnLine(x, y, arg, line);

        if ((binarized_line[0] == line[0]) && (binarized_line[1] == line[1]))
        {
            line_num++;
            sum_line[0] += x;
            sum_line[0] += y;
            sum_line[0] += arg;
        }
    }

    if (line_num != 0)
    {
        for (int i = 0; i < 3; i++)
            trueth_pose[i] = sum_line[i] / line_num;
    }
}

// 初期化位置の生成
geometry_msgs::PoseWithCovarianceStamped generateInitialpose(float x, float y, float yaw)
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0;
    pose.pose.pose.orientation = toQuaternion(0, 0, yaw);

    // pose.pose.covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.006853891945200942};

    pose.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_localization_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    // amclからのデータ受信者
    ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, callbackAmcl);
    ros::Subscriber sub_reset = nh.subscribe("reset", 100, callbackReset);
    ros::Subscriber sub_odom_data = nh.subscribe("odom_data", 100, callbackOdomData);
    ros::Subscriber sub_line_x = nh.subscribe("line_sensor/binarized/x", 100, callbackLineX);
    ros::Subscriber sub_line_y = nh.subscribe("line_sensor/binarized/y", 100, callbackLineY);
    ros::Subscriber sub_particle = nh.subscribe("particlecloud", 100, callbackparticlecloud);

    // 位置初期化データの送信者
    ros::Publisher pub_initialpose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
    ros::Publisher pub_robot_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 10);

    while (ros::ok())
    {
        if (needs_reset)
        {
            pub_initialpose.publish(generateInitialpose(odom_data[0], odom_data[1], odom_data[2]));
            needs_reset = false;
            // test: rostopic pub /reset std_msgs/Bool true
        }

        // ROS_INFO("%f %f %d \r\n", sensor_pose[0],  sensor_pose[1], line[0]);]

        if (binarized_line[0] != pre_binarized_line[0] || binarized_line[1] != pre_binarized_line[1])
        {
            // pub_initialpose.publish(generateInitialpose(trueth_pose[0], trueth_pose[1], trueth_pose[2]));
        }

        pre_binarized_line[0] = binarized_line[0];
        pre_binarized_line[1] = binarized_line[1];

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
