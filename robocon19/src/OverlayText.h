void configOverlayText(jsk_rviz_plugins::OverlayText &t, std::string str);
std::string generateDisplayStr(float pose[3]);

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
