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
    char str_1[20] = (switch_val & 0b00000001) ? "昇降位置合わせ" : "通常走行";
    char str_3[20] = (switch_val & 0b00000010) ? "青" : "赤";
    char str_4[20] = (switch_val & 0b00000100) ? "決勝" : "予選";
    char str_5[20] = (switch_val & 0b00001000) ? "バスタオル:掛ける" : "バスタオル:掛けない";
    char str_6[20] = (switch_val & 0b00110000 == 0) ? "シーツ:掛けない" : ((switch_val & 0b00110000 == 1) ? "シーツ:3点" : "シーツ:9点");

    sprintf(display_chars, "%d\r\n\
                            ____amcl_____odom_____diff_\r\n\
                            x| %+.3f | %+.3f | %+.3f\r\n\
                            y| %+.3f | %+.3f | %+.3f\r\n\
                            z| %+.3f | %+.3f | %+.3f",
            switch_val, pose[0], odom[0], pose[0] - odom[0], pose[1], odom[1], pose[1] - odom[1], pose[2] * (180 / 3.14159), odom[2] * (180 / 3.14159), (pose[2] - odom[2]) * (180 / 3.14159));
}
