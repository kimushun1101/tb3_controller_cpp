# tb3_controller_cpp

Turtlebot3に制御則を実装するパッケージ．

## 環境構築

1. Ubuntu22.04を用意する．  
   WSL でも可．
2. Git とROS2の環境を準備する．  
   [非公式インストールスクリプト](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu)が便利
   ```
   sudo apt update
   sudo apt install git
   git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu
   cd ros2_setup_scripts_ubuntu
   ./run.sh
   ```
3. ROS2のワークスペースを作り，このパッケージをインストールする．
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/kimushun1101/tb3_controller_cpp.git
   ```
4. このパッケージの依存関係を解決する．
   ```
   rosdep update
   cd ~/ros2_ws
   rosdep install -y --from-paths src
   ```
5. ビルドする．
   ```
   cd ~/ros2_ws
   colcon build
   ```

## 実行方法

1. シミュレータを起動する．
   ```
   export TURTLEBOT3_MODEL=burger
   export LIBGL_ALWAYS_SOFTWARE=1  # オンボードGPU のときはこれをしないとGazebo が暗くなる
   ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
   ```
   初回はエラーが出てロボットモデルが出ないが，`Ctrl+c`で一度閉じ，再度
   ```
   ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
   ```
   すると正しく起動する．
2. 別のターミナルで制御を開始する．
    ```
   source ~/ros2_ws/install/setup.bash
   ros2 run tb3_controller_cpp tb3_controller_node
   ```

## パラメータ調整方法
1. Gazebo 上のロボットの移動は，`t`キーを押下して`Translation Mode`に移行してからロボットをドラッグ・アンド・ドロップ．  
   シミュレーションをリセットしたい場合には以下のROS 2 service コマンドを実行する．
   ```
   ros2 service call /reset_simulation std_srvs/srv/Empty
   ```
2. 目標値の変更
   ```
   ros2 topic pub /xd std_msgs/msg/Float32 "data: 3.0"
   ```
3. パラメータを調整して実行するためには，一度`Ctrl+c` で制御則を切り，以下で実行し直す．
   ```
   ros2 run tb3_controller_cpp tb3_controller_node --ros-args -p Kp:=3.0
   ```
   `Kp`，`Kd`，および`T`を色々変えて実行してみよう．
   目標値は手順2 でも変更できるが，起動時の目標値として`init_xd` というパラメータも用意している．  
   都度`-p` オプションをつければ，複数のパラメータを同時に設定することもできる．
   ```
   ros2 run tb3_controller_cpp tb3_controller_node --ros-args -p Kp:=2.0 -p Kd:=1.5 -p T:=0.01 -p init_xd:=1.5
   ```
4. データの記録
   rosbag2 を使用してデータの記録を行う．
   ```
   cd ~/ros2_ws/src/tb_controller_cpp/result
   ros2 bag record /scan /xd /cmd_vel
   ```
5. グラフを書く
   ```
   ros2 run plotjuggler plotjuggler
   ```
   `File`→`Data`からrosbag2 で保存したデータを読み込み描画する．  
   自身のプログラムでグラフ作成したい場合には，CSV Exporter を使用すればCSV 形式でも取得できる．

