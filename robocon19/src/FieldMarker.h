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
