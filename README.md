# KOSEN Robocon 2019 ROS programs
=================

# 必要なデバイス
* YDLIDAR X4（以下LRF）

# 動作環境
* Ubuntu 16.04 （LTS）
* ROS Kinetic
* Intel Core i7 5500U

# ROS ノード一覧
このプロジェクトは以下のノードを含みます。

* robocon19_lrf_node - LRFのデータを出版する


## 自己位置推定シミュレーションの実行
ターミナルで
```shell
roslaunch robocon19_sim_stage robocon19_sim_stage.launch
```
デフォルトは赤ゾーンでの起動となる

デフォルトは青ゾーンで起動したい場合は
```shell
roslaunch robocon19_sim_stage robocon19_sim_stage.launch zone:=blue
```

## LRFプログラムの実行
[EAIの公式が出してるコード](https://github.com/EAIBOT/ydlidar)を書き換えただけ

### udevの追加（初回のみ）
LRFのシリアルポートを「/dev/ydlidar」と登録する．

LRFのみを接続した状態で以下を実行する．
```shell
cd ~/ros_catkin_ws/src/ydlidar/startup
chmod 777 *
sudo sh initenv.sh
```

`ls /dev/ydlidar`で見れれば成功．

### プログラムの実行
```shell
roslaunch ydlidarrobocon19_lrf lidar_view.launch
```
rvizも起動し，点群が確認できる．

可視化が必要ない場合は
```shell
roslaunch ydlidarrobocon19_lrf lidar.launch
```

### 公式のコードとの変更点
主にgmappingとの接続性を向上させるための変更を行った

#### frame_id
laser_frame　→　base_laser_link

#### topic名
/scan　→　base_scan


#LICENSE

Copyright (c) 2015 Ryoga Sato

https://surpace0924.github.io/

Released under the MIT license