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
sudo apt-get install -y ros-kinetic-jsk-visualization
```

# ROS ノード一覧
このプロジェクトは以下のノードを含みます。

* robocon19_node - 全ノードの統括ノード
* robocon19_lrf_node - LRFのデータを出版する


## 自己位置推定シミュレーションの実行
ターミナルで以下を実行．（デフォルトは赤ゾーンでの起動となる）
```shell
roslaunch robocon19_sim_stage sim_stage.launch
```

青ゾーンで起動したい場合は
```shell
roslaunch robocon19_sim_stage sim_stage.launch zone:=blue
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
rvizも起動し，点群が確認できる．
```shell
roslaunch ydlidarrobocon19_lrf lidar_view.launch
```

可視化が必要ない場合は
```shell
roslaunch ydlidarrobocon19_lrf lidar.launch
```

### 公式のコードとの変更点

#### frame_id
`/laser_frame`　→　`/laser`

# LICENSE
Copyright (c) 2019 Ryoga Sato

https://surpace0924.github.io/

Released under the MIT license