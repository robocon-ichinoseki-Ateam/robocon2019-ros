visualization_msgs::MarkerArray generateDisplayLinesensor(int sensor_id, int snesor_val)
{
    visualization_msgs::MarkerArray m_a;

    float attach_pose[2][2] = {{0, -0.295}, {-0.295, 0}};

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
            m_a.markers[i].pose.position.x = attach_pose[sensor_id][0] + (float)i * interval - interval * 3.5;
            m_a.markers[i].pose.position.y = attach_pose[sensor_id][1];
        }
        else
        {
            m_a.markers[i].pose.position.x = attach_pose[sensor_id][0];
            m_a.markers[i].pose.position.y = attach_pose[sensor_id][1] + (float)i * interval - interval * 3.5;
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

visualization_msgs::Marker generateDisplayRobotFootprint()
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

visualization_msgs::Marker generateDisplayRobotLift(float lift_height)
{
    visualization_msgs::Marker mk;
    mk.id = 101;
    mk.header.frame_id = "/base_link";
    mk.header.stamp = ros::Time::now();
    mk.ns = "robocon19_node";
    mk.lifetime = ros::Duration();
    mk.type = visualization_msgs::Marker::CUBE;
    mk.scale.x = 0.5;
    mk.scale.y = 0.5;
    mk.scale.z = lift_height;

    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = lift_height / 2;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    mk.color.b = 1.0;
    mk.color.a = 0.5;

    return mk;
}
