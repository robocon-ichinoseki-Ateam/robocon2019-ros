# KOSEN Robocon 2019 ROS programs

# 動作環境
* Ubuntu 16.04 （LTS）
* ROS Kinetic
* Intel Core i7 5500U

# 必要なデバイス
* YDLIDAR X4（以下LRF）
* メインマイコンとの通信用マイコン（LPC1768）
* ラインセンサ用マイコン（NUCLEO F303K8）

# 必要なライブラリ
rviz上に情報を表示するために、jskのrvizプラグラインを使用している．
kinetic版は以下のコマンドでインストールできる．
```shell
$ sudo apt-get install -y ros-kinetic-jsk-visualization
```

シミュレータではPS3コントローラを接続できる．<br>
コントローラのプログラムインストールは以下のコマンドで行える．
``` shell
$ sudo apt-get install ros-kinetic-joy
$ sudo apt-get install ros-kinetic-joystick-drivers
```

その他、自己位置推定に必要なライブラリ群は以下のコマンドでインストールできる．
```shell
$ sudo apt update
$ sudo apt upgrade
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers ros-kinetic-scan-tools
```

# ディレクトリ一覧
## robocon19
全ノードの統括

## robocon19_bag
bagファイルの保存

## robocon19_diagnostics
Topicの送信状態を監視してdiagnosticsを配信する

## robocon19_laser_filter
LRFの不要なデータをカットする（不具合発生のため使ってない．）．

## robocon19_laser_odometry
LRFのスキャンデータからオドメトリのデータを生成する（オドメータのデータが得られない時用．本番では使用しない．）．

## robocon19_lrf
LRFのデータを出版する．

## robocon19_map
mapファイルの保存

## robocon19_mbed
mbedとの通信処理．<br>
mbedからのFloat配列のtopicをROS内通信でのtopicに直したり，mbedにデータを送信したりする

## robocon19_localization
自己位置推定を行う．

## robocon19_sim_stage
2Dシミュレータのstageを使用して値の入出力を確認できる．


# トピック一覧
## /mbed_to_ros
## /ros_to_mbed
## /line/x
## /line/y
## /reset


<!-- # graph
## debugも表示したもの
![rosgraph3](https://user-images.githubusercontent.com/25032035/59808086-a27a8900-9335-11e9-8127-b2b22c563ee4.png)

## debugを省略したやつ
![](https://user-images.githubusercontent.com/25032035/59808016-5e878400-9335-11e9-97d2-a9c35c3bf809.png) -->


# 初回にやること
## udevの追加
LRFのシリアルポートを「/dev/ydlidar」と登録する．

LRFのみを接続した状態で以下を実行する．
```shell
$ cd ~/catkin_ws/src/robocon2019-ros/robocon19_lrf/startup
$ chmod 777 *
$ sudo sh initenv.sh
```

`ls /dev/ydlidar`で見れれば成功．


# 起動
## オプション
### zone
フィールドを選択できる.
* red: 赤ゾーン
* blue: 青ゾーン
* test: 情報演習室前廊下
* step: デザイン室前階段付近の廊下

### mode
通常モードとデバッグモードの選択ができる．
* nomal: 通常モード（mbedとLRFを接続して実機で動かすモード）
* debug: デバッグモード（bagファイルを使用した実機が不要のコード確認用モード）
* sim: シミュレータモード

### bag
デバッグモードの際に呼び出すbagファイルを選択する．

## 通常モードでの起動
1. PCにmbedとLRFを接続．
2. mbedにアクセス権限を付与．
```shell
$ sudo chmod a+rw /dev/ttyACM0
$ sudo chmod a+rw /dev/ttyACM1
$ sudo chmod a+rw /dev/ttyACM2
```

3. launchを立ち上げる．（下は赤ゾーンの例）
```shell
$ roslaunch robocon19 robocon19.launch zone:=red
```

## デバッグモードでの起動
modeオプションをdebugにする。
```shell
$ roslaunch robocon19 robocon19.launch zone:=step mode:=debug bag:=0617/1
```
「bag:=hoge」と指定すると，「robocon19_bag/zone/hoge.bag」のbagファイルが読み込まれる．<br>
そのため，bagを保存する際にはrobocon19_bagの直下にはゾーン名に対応したディレクトリを作成する必要がある．

## シミュレータの起動
ターミナルで以下を実行．（デフォルトは赤ゾーンでの起動となる）
```shell
$ roslaunch robocon19 robocon19.launch mode:=sim
```

# LICENSE
Copyright (c) 2019 Ryoga Sato

https://surpace0924.github.io/

Released under the MIT license
