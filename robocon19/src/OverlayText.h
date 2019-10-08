char display_chars[256];
void configOverlayText(jsk_rviz_plugins::OverlayText &t);
void generateDisplayStr(float pose[3]);

// rvizに表示するテキストの設定
void configOverlayText(jsk_rviz_plugins::OverlayText &t)
{
    t.action = jsk_rviz_plugins::OverlayText::ADD;
    t.width = 230;
    t.height = 78;
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
    t.text_size = 13;
    t.font = "Cica";
    t.text = display_chars;
}

// rvizに表示するテキストデータを生成
void generateDisplayStr(float pose[3], float odom[3])
{
    sprintf(display_chars, "____amcl_____odom_____diff_\r\n\
                            x| %+.3f | %+.3f | %+.3f\r\n\
                            y| %+.3f | %+.3f | %+.3f\r\n\
                            z| %+.3f | %+.3f | %+.3f",
            pose[0], odom[0], pose[0] - odom[0], pose[1], odom[1], pose[1] - odom[1], pose[2] * (180 / 3.14159), odom[2] * (180 / 3.14159), (pose[2] - odom[2]) * (180 / 3.14159));
}
