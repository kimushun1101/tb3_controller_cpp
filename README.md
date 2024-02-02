# tb3_controller_cpp

Turtlebot3 に制御則を実装するパッケージ．  
ロボットとその後方にある壁との距離を，指定した目標値に安定化する P 制御器を実装している．

## 環境構築

1. Ubuntu 22.04を用意  
   WSL でも可能であることは確認している．
2. ROS 2 環境構築  
   [公式インストールページ](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) を参考に ROS 2 をインストールして，
   [Building a Custom Debian Package](https://docs.ros.org/en/humble/How-To-Guides/Building-a-Custom-Debian-Package.html) を参考に `rosdep` の初期化まで完了させておく．  
   さらに，`~/.bashrc` に ROS コマンドを有効にするためのコマンドを追加する．
   ```
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

## パッケージのインストールとビルド

1. ROS2のワークスペースを作り，このパッケージをインストール
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   sudo apt update
   sudo apt install git
   git clone https://github.com/kimushun1101/tb3_controller_cpp.git
   ```
2. このパッケージの依存関係を解決
   ```
   cd ~/ros2_ws
   rosdep install -y --from-paths src
   ```
3. ビルド
   ```
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

## シミュレータと制御則の実行

1. シミュレータの起動
   ```
   # Terminal 1
   export LIBGL_ALWAYS_SOFTWARE=1  # オンボードGPU のときはこれをしないとGazebo が暗くなる？
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
   ```
   初回時はGazebo の立ち上がりが遅く，エラーが出てロボットモデルが出ないかもしれない．  
   そのような場合には`Ctrl+C` で一度閉じ，再度
   `ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py`
   を実行する．  
   それでもロボットモデルが出ない場合には，Gazebo 画面内の左にあるInsert タブから，Turtlebot3(Burger) をクリックしてシミュレータ上にロボットを手動で置く．
2. 新しく別のターミナルを開き，以下のコマンドで制御を開始（コントローラーを実行）
   ```
   # Terminal 2
   source ~/ros2_ws/install/setup.bash
   ros2 run tb3_controller_cpp tb3_controller_node
   ```

シミュレータと制御則を 1 つのターミナルから同時実行させたい場合には，上記の 2 つのターミナルのコマンドを`Ctrl+C` で終了して，以下のコマンドを入力する．
```
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 launch tb3_controller_cpp simulation_and_controller.launch.yaml 
```
`Ctrl+C` でシミュレータも終了してしまうため，以下のパラメーター調整には不向き．

## パラメーター調整

`# Terminal 1` でシミュレータ，`# Terminal 2` でコントローラーを起動しているものとする．

1. Gazebo 上のロボットの移動  
   `t` キーを押下して `Translation Mode` に移行してからロボットをドラッグ・アンド・ドロップ．  
   シミュレーションをリセットしたい場合には以下の ROS 2 service コマンドを実行する．
   ```
   # Terminal 3
   ros2 service call /reset_simulation std_srvs/srv/Empty
   ```
2. 目標値の変更  
   以下のコマンドを実行する．
   ```
   # Terminal 3
   ros2 topic pub /xd std_msgs/msg/Float32 "data: 3.0"
   ```
   新しい目標値に向かってロボットが動くはず．
3. パラメーターの調整  
   制御則を実行したターミナル `# Terminal 2` で `Ctrl+C` を押下することで制御則を一度切り，以下で実行し直す．
   ```
   # Terminal 2
   ros2 run tb3_controller_cpp tb3_controller_node --ros-args -p Kp:=3.0
   ```
   `Kp` と `T` を色々変えて実行してみよう．
   目標値は手順2 でも変更できるが，起動時の目標値として `init_xd` というパラメーターも用意している．  
   都度 `-p` オプションをつければ，複数のパラメーターを同時に設定することもできる．
   ```
   # Terminal 2
   ros2 run tb3_controller_cpp tb3_controller_node --ros-args -p Kp:=2.0 -p T:=0.01 -p init_xd:=1.5
   ```

## 結果出力

1. データの記録  
   rosbag2 を使用してデータの記録を開始する．
   ```
   # Terminal 3
   mkdir -p ~/ros2_ws/src/tb3_controller_cpp/result
   cd ~/ros2_ws/src/tb3_controller_cpp/result
   ros2 bag record /scan /xd /cmd_vel
   ```
   `# Terminal 2` で記録したい制御則を実行し，`# Terminal 3` を `Ctrl+C` することで記録を終了する．
2. グラフを書く
   ```
   # Terminal 3
   ros2 run plotjuggler plotjuggler
   ```
   `File`→`Data` から rosbag2 で保存したデータ `metadata.yaml` を読み込み描画する．  
   自身のプログラムでグラフ作成したい場合には，CSV Exporter を使用すれば CSV 形式でも取得できる．

## Turtlebot 3 実機での実行

1. ロボットのセットアップ  
   [公式の e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) に従う
2. ロボットのソフトウェアを立ち上げ
   ```
   # SSH raspberry Pi 1
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_bringup robot.launch.py 
   ```
3. 新しく別のターミナルを開き，以下のコマンドで制御を開始
   ```
   # SSH raspberry Pi 2
   source ~/ros2_ws/install/setup.bash
   ros2 run tb3_controller_cpp tb3_controller_node
   ```

シミュレータと制御則を 1 つのターミナルから同時実行させたい場合には，以下のコマンドを入力する．
```
# SSH raspberry Pi 1
source ~/ros2_ws/install/setup.bash
ros2 launch tb3_controller_cpp turtlebot3_and_controller.launch.yaml 
```
