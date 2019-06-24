#include "../../util/util.h"

int state_amcl = 2;
int state_line_sensor = 2;

void diagnosticRobot(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    int state_sum = 0;
    state_sum += state_amcl;
    state_sum += state_line_sensor;

    if (state_sum == 0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Robot OK");
    }
    else
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Robot Error");
    }
}

void callbackAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL)
{
    state_amcl = 0;
}
void diagnosticAmcl(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (state_amcl == 0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "publish OK");
    }
    else if (state_amcl == 1)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "publish Warn");
    }
    else
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "publish Error");
    }
}

void callbackLinesensor(const std_msgs::Int32MultiArray &msg_line_sensor)
{
    state_line_sensor = 0;
}
void diagnosticLinesensor(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if (state_line_sensor == 0)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "publish OK");
    }
    else if (state_line_sensor == 1)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "publish Warn");
    }
    else
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "publish Error");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_diagnostics_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    diagnostic_updater::Updater updater;

    updater.setHardwareID("Robot");
    updater.add("robot", diagnosticRobot);

    ros::Subscriber sub_amcl = nh.subscribe("amcl_pose", 100, callbackAmcl);
    updater.setHardwareID("AMCL");
    updater.add("AMCL", diagnosticAmcl);

    ros::Subscriber sub_line_sensor = nh.subscribe("line_sensor", 100, callbackLinesensor);
    updater.setHardwareID("LineSensor");
    updater.add("LineSensor", diagnosticLinesensor);

    while (ros::ok())
    {
        updater.update();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
