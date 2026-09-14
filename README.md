# ROS 2 Nav2 TurtleBot3 多点巡航

路线 A：在 Gazebo + Nav2 仿真里给 TurtleBot3 依次发多个导航目标，并对比速度参数。

**不是** YOLO 检测、也不是机械臂抓取。那些是另一条线（桌面臂仿真）。

## 环境

- Ubuntu 24.04
- ROS 2 Jazzy
- 软件包：`ros-jazzy-navigation2`、`ros-jazzy-nav2-bringup`、`ros-jazzy-turtlebot3-gazebo`（名称以你系统里实际包为准）

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
```

克隆后在仓库根目录执行下面的脚本即可，不必改成家目录路径。

## 0. 启动仿真（终端 1，一直挂着）

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py use_composition:=False headless:=False
```

应弹出 Gazebo 和 RViz。

## 1. 设初始位姿（终端 2）

仓库根目录：

```bash
source /opt/ros/jazzy/setup.bash
bash ./00_set_initial_pose.sh
```

看 RViz：车应在地图左下附近。

## 2. 激活导航（若 Startup 无效 / 面板 inactive）

```bash
bash ./01_activate_nav.sh
```

看到 `/bt_navigator: active` 即可。

## 3. 多点自动巡航

```bash
source /opt/ros/jazzy/setup.bash
python3 ./02_multi_goals.py
```

车会依次走脚本里的路点。改 `02_multi_goals.py` 顶部的 `WAYPOINTS` 即可换点。

## 4. 调参对比

```bash
bash ./03_set_max_vel.sh 0.10
python3 ./02_multi_goals.py

bash ./03_set_max_vel.sh 0.26
python3 ./02_multi_goals.py
```

对比快慢。笔记写清：改了哪个参数、现象是什么。

## 面试一句话

基于 ROS 2 Nav2 仿真完成多点巡航脚本，并对比控制器速度参数对行为的影响；理解定位–规划–控制接口，而不是从零实现导航算法。
