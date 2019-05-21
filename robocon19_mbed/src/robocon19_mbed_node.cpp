#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double state_odom_x = 0.0;  //オドメトリX座標[m]
double state_odom_y = 0.0;  //オドメトリY座標[m]
double state_odom_th = 0.0; //オドメトリ姿勢[rad]

void chatterCallback(const std_msgs::Float32MultiArray &msg)
{
    int num = msg.data.size();
    state_odom_x = msg.data[0];
    state_odom_y = msg.data[1];
    state_odom_th = msg.data[2];

    ROS_INFO("I susclibed [%i]", num);
    for (int i = 0; i < num; i++)
    {
        ROS_INFO("[%i]:%f", i, msg.data[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocon19_mbed_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("mbed_to_ros", 1000, chatterCallback);

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    current_time = ros::Time::now();

    ros::Rate r(30.0);

    while (nh.ok())
    {
        current_time = ros::Time::now();

        //tf odom->base_link
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = state_odom_x;
        odom_trans.transform.translation.y = state_odom_y;
        odom_trans.transform.translation.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state_odom_th);
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        ros::spinOnce();
        r.sleep();
    }
}

// #include "ros/ros.h"             // rosで必要はヘッダーファイル
// #include <geometry_msgs/Twist.h> // ロボットを動かすために必要
// using namespace std;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "my_teleop_node");
//     // initでROSを初期化し、my_teleop_nodeという名前をノードにつける
//     // 同じ名前のノードが複数あるとだめなので、ユニークな名前をつける

//     ros::NodeHandle nh;
//     // ノードハンドラの作成。ハンドラは必要時に起動される。

//     ros::Publisher pub;
//     // パブリッシャの作成。トピックに対してデータを送信。

//     ros::Rate rate(10);
//     // ループの頻度を設定するためのオブジェクトを作成。この場合は10Hz、1秒間に10回数、1ループ100ms。

//     geometry_msgs::Twist vel;
//     // geometry_msgs::Twist　この型は並進速度と回転速度(vector3:3次元ベクトル) を合わせたもので、速度指令によく使われる

//     pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
//     // マスターにgeometry_msgs::Twist型のデータを送ることを伝える
//     // マスターは/cmd_velトピック(1番目の引数）を購読する
//     // 全てのノードにトピックができたことを知らせる(advertise)。
//     // 2番目の引数はデータのバッファサイズ

//     cout << "f: fompu6050_serial_to_imu_noderward, b: backward, r: right, l:left" << endl;

//     while (ros::ok())
//     {             // このノードが使える間は無限ループする
//         char key; // 入力キーの値

//         cin >> key;
//         　 // 標準入力からキーを読み込む
//             cout
//             << key << endl; // 読み込んだキーの値を標準出力へ出力

//         switch (key)
//         {
//         case 'f': // fキーが押されていたら
//             vel.linear.x = 0.5;
//             break;
//         case 'b':
//             vel.linear.x = -0.5;
//             break;
//         case 'l':
//             vel.angular.z = 1.0;
//             break;
//         case 'r':
//             vel.angular.z = -1.0;
//             break;
//             // linear.xは前後方向の並進速度(m/s)。前方向が正。
//             // angular.zは回転速度(rad/s)。反時計回りが正。
//         }

//         pub.publish(vel);    // 速度指令メッセージをパブリッシュ（送信）
//         ros::spinOnce();     // １回だけコールバック関数を呼び出す
//         vel.linear.x = 0.0;  // 並進速度の初期化
//         vel.angular.z = 0.0; // 回転速度の初期化
//         rate.sleep();        // 指定した周期でループするよう寝て待つ
//     }

//     return 0;
// }
