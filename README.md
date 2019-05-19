# KOSEN Robocon 2019 ROS programs

# 動作環境
* Ubuntu 16.04 （LTS）
* ROS Kinetic
* Intel Core i7 5500U

# 必要なデバイス
* YDLIDAR X4（以下LRF）

# 必要なライブラリ
rviz上に情報を表示するために、jskのrvizプラグラインを使用しています。
kinetic版は以下のコマンドでインストールできます。
```shell
$ sudo apt-get install -y ros-kinetic-jsk-visualization
```

その他、自己位置推定に必要なライブラリ群は以下のコマンドでインストールできます。
```shell
$ sudo apt update
$ sudo apt upgrade
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers ros-kinetic-scan-tools
```

# ROS ノード一覧
このプロジェクトは以下のノードを含みます。

* robocon19_node - 全ノードの統括ノード
* robocon19_lrf_node - LRFのデータを出版する

## 自己位置推定シミュレーションの実行
ターミナルで以下を実行．（デフォルトは赤ゾーンでの起動となる）
```shell
$ roslaunch robocon19_sim_stage sim_stage.launch
```

青ゾーンで起動したい場合は
```shell
$ roslaunch robocon19_sim_stage sim_stage.launch zone:=blue
```

## LRFプログラムの実行
[EAIの公式が出してるコード](https://github.com/EAIBOT/ydlidar)を書き換えただけ

### udevの追加（初回のみ）
LRFのシリアルポートを「/dev/ydlidar」と登録する．

LRFのみを接続した状態で以下を実行する．
```shell
$ cd ~/catkin_ws/src/robocon2019-ros/robocon19_lrf/startup
$ chmod 777 *
$ sudo sh initenv.sh
```

`ls /dev/ydlidar`で見れれば成功．

### プログラムの実行
rvizも起動し，点群が確認できる．
```shell
$ roslaunch ydlidarrobocon19_lrf lidar_view.launch
```

可視化が必要ない場合は
```shell
$ roslaunch ydlidarrobocon19_lrf lidar.launch
```

### 公式のコードとの変更点

#### frame_id
`/laser_frame`　→　`/laser`

# LICENSE
Copyright (c) 2019 Ryoga Sato

https://surpace0924.github.io/

Released under the MIT license
