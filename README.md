# tb3_controller_cpp

Turtlebot3に制御則を実装するパッケージ。

## 環境構築

1. Ubuntu22.04を用意する。  
    WSL でも可。
2. Git とROS2の環境を準備する。  
    [非公式インストールスクリプト](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu)が便利
    ```
    sudo apt update
    sudo apt install git
    git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu
    cd ros2_setup_scripts_ubuntu
    ./run.sh
    ```
3. ROS2のワークスペースを作り、このパッケージをインストールする。
   ```
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/kimushun1101/tb3_controller_cpp.git
   ```
4. このパッケージの依存関係を解決する。
   ```
    sudo rosdep init
    rosdep update
    cd ~/ros2_ws/src
    rosdep install -y --from-paths src
   ```
5. ビルドする。
   ```
    cd ~/ros2_ws/src
    colcon build
   ```
6. シミュレータを起動する。
   ```
    cd ~/ros2_ws/src
    source install/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
7. 別のターミナルで制御を開始する。
    ```
    cd ~/ros2_ws/src
    source install/setup.bash
    ros2 run tb3_controller_cpp tb3_controller_cpp
   ```
