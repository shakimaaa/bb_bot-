# bb_bot
ros2_control下的基于Nav2的自动导航机器人

## 环境要求
- 我在以下版本进行测试
    - Ubuntu 20.04
    - ROS2 foxy
    - ros2_control

## 构建
1. 克隆这个仓库到你的工作空间
```
$ mkdir -p colcon_ws/src
$ cd ~/colcon_ws/src
$ git clone https://github.com/shakimaaa/bb-robot.git
```

2. 更改这里的包名 [line](bb_bot/launch/launch_sim_test.launch.py#L20) 为你自己的

3. build
```
$ cd ~/colcon_ws
$ colcon build --symlink-install 
```

## 启动仿真
```
$ source intall/setup.bash
$ ros2 launch bb_bot launch_sim_test.launch.py
```

## 启动雷达
```
$ source intall/setup.bash
$ ros2 launch ydlidar ydlidar_launch.py
```
