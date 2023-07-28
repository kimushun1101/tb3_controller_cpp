# tb3_controller_cpp

Turtlebot3 に制御則を実装するパッケージ．  
ロボットとその後方にある壁との距離を，指定した目標値に安定化するPD制御器を実装している．

## 環境構築

1. Ubuntu22.04を用意  
   WSL でも可能であることは確認している．
2. Git のインストールとROS 2 環境の準備  
   [非公式インストールスクリプト](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu)が便利
   ```
   sudo apt update
   sudo apt install git
   git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu
   cd ros2_setup_scripts_ubuntu
   ./run.sh
   ```
3. ROS2のワークスペースを作り，このパッケージをインストール
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/kimushun1101/tb3_controller_cpp.git
   ```
4. このパッケージの依存関係を解決
   ```
   rosdep update
   cd ~/ros2_ws
   rosdep install -y --from-paths src
   ```
5. ビルド
   ```
   cd ~/ros2_ws
   colcon build
   ```

## 実行方法

1. シミュレータの起動
   ```
   export TURTLEBOT3_MODEL=burger
   export LIBGL_ALWAYS_SOFTWARE=1  # オンボードGPU のときはこれをしないとGazebo が暗くなる？
   ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
   ```
   初回時，エラーが出てロボットモデルが出ない場合には，`Ctrl+c`で一度閉じ，再度以下を実行
   ```
   ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
   ```
   それでもロボットモデルが出ない場合には，Gazebo 画面内の左にあるInsert タブから，Turtlebot3(Burger) をクリックしてシミュレータ上にロボットを手動で置く．
2. 新しく別のターミナルを開き，以下のコマンドで制御を開始
    ```
   source ~/ros2_ws/install/setup.bash
   ros2 run tb3_controller_cpp tb3_controller_node
   ```

## 調整方法
1. Gazebo 上のロボットの移動  
   `t`キーを押下して`Translation Mode`に移行してからロボットをドラッグ・アンド・ドロップ．  
   シミュレーションをリセットしたい場合には以下のROS 2 service コマンドを実行する．
   ```
   ros2 service call /reset_simulation std_srvs/srv/Empty
   ```
2. 目標値の変更
   別のターミナルを開いて以下のコマンドを実行する．
   ```
   ros2 topic pub /xd std_msgs/msg/Float32 "data: 3.0"
   ```
   新しい目標値に向かってロボットが動くはず．
3. パラメータの調整  
   制御則を実行したターミナルで`Ctrl+c` を押下することで制御則を一度切り，以下で実行し直す．
   ```
   ros2 run tb3_controller_cpp tb3_controller_node --ros-args -p Kp:=3.0
   ```
   `Kp`，`Kd`，および`T`を色々変えて実行してみよう．
   目標値は手順2 でも変更できるが，起動時の目標値として`init_xd` というパラメータも用意している．  
   都度`-p` オプションをつければ，複数のパラメータを同時に設定することもできる．
   ```
   ros2 run tb3_controller_cpp tb3_controller_node --ros-args -p Kp:=2.0 -p Kd:=1.5 -p T:=0.01 -p init_xd:=1.5
   ```

## 結果出力
1. データの記録  
   rosbag2 を使用してデータの記録を行う．
   ```
   cd ~/ros2_ws/src/tb_controller_cpp/result
   ros2 bag record /scan /xd /cmd_vel
   ```
2. グラフを書く
   ```
   ros2 run plotjuggler plotjuggler
   ```
   `File`→`Data`からrosbag2 で保存したデータを読み込み描画する．  
   自身のプログラムでグラフ作成したい場合には，CSV Exporter を使用すればCSV 形式でも取得できる．

