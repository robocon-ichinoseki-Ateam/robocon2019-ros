#include "../../util/util.h"

#define device_num 7

enum
{
    amcl_id,
    line_sensor_id,
    scan_id,
    map_id,
    to_ros_id,
    to_mbed_id,
    costmap_id
};

int state[device_num] = {2};

void diagnosticRobot(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    int state_sum = 0;
    for (int i = 0; i < device_num; i++)
    {
        state_sum += state[i];
    }

    if (state_sum == 0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Robot OK");
    }
    else
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Robot Error");
    }
}

void generateDiagnosticupdater(int val, diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (val == 0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "publish OK");
    }
    else if (val == 1)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "publish Warn");
    }
    else
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "publish Error");
    }
}

void callbackAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
{
    state[amcl_id] = 0;
}
void diagnosticAmcl(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    generateDiagnosticupdater(state[amcl_id], stat);
}

void callbackLinesensor(const std_msgs::Int32MultiArray &msg_line_sensor)
{
    state[line_sensor_id] = 0;
}
void diagnosticLinesensor(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    generateDiagnosticupdater(state[line_sensor_id], stat);
}

void callbackScan(const sensor_msgs::LaserScan &msg_scan)
{
    state[scan_id] = 0;
}
void diagnosticScan(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    generateDiagnosticupdater(state[scan_id], stat);
}

void callbackMap(const nav_msgs::OccupancyGrid &msg_map)
{
    state[map_id] = 0;
}
void diagnosticMap(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    generateDiagnosticupdater(state[map_id], stat);
}

void callbackToros(const std_msgs::Float32MultiArray &msg)
{
    state[to_ros_id] = 0;
}
void diagnosticToros(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    generateDiagnosticupdater(state[to_ros_id], stat);
}

void callbackTombed(const std_msgs::Float32MultiArray &msg)
{
    state[to_mbed_id] = 0;
}
void diagnosticTombed(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    generateDiagnosticupdater(state[to_mbed_id], stat);
}

void callbackCostmap(const nav_msgs::OccupancyGrid &msg)
{
    state[costmap_id] = 0;
}
void diagnosticCostmap(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    generateDiagnosticupdater(state[costmap_id], stat);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_diagnostics_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    for (int i = 0; i < device_num; i++)
    {
        state[i] = 2;
    }
    state[1] = 0;
    state[5] = 0;
    state[6] = 0;

    diagnostic_updater::Updater updater;

    updater.setHardwareID("Robot");
    updater.add("Robot", diagnosticRobot);

    ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, callbackAmcl);
    updater.setHardwareID("AMCL");
    updater.add("AMCL", diagnosticAmcl);

    ros::Subscriber sub_line_sensor = nh.subscribe("line_sensor", 100, callbackLinesensor);
    updater.setHardwareID("LineSensor");
    updater.add("LineSensor", diagnosticLinesensor);

    ros::Subscriber sub_scan = nh.subscribe("scan", 100, callbackScan);
    updater.setHardwareID("LRF");
    updater.add("LRF", diagnosticScan);

    ros::Subscriber sub_map = nh.subscribe("map", 100, callbackMap);
    updater.setHardwareID("Map");
    updater.add("Map", diagnosticMap);

    ros::Subscriber sub_to_ros = nh.subscribe("mbed_to_ros", 100, callbackToros);
    updater.setHardwareID("mbed_to_ros");
    updater.add("mbed_to_ros", diagnosticToros);

    ros::Subscriber sub_to_mbed = nh.subscribe("ros_to_mbed", 100, callbackTombed);
    updater.setHardwareID("ros_to_mbed");
    updater.add("ros_to_mbed", diagnosticTombed);

    ros::Subscriber sub_costmap = nh.subscribe("/move_base_node/global_costmap/costmap", 100, callbackCostmap);
    updater.setHardwareID("Costmap");
    updater.add("Costmap", diagnosticCostmap);

    while (ros::ok())
    {
        updater.update();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
