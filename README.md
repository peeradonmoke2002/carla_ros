# ROS/ROS2 bridge for CARLA simulator

This is a fork of [carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge) to adapt ros humble and carla 0.9.15

### Quickly Start

Environment

* OS: ubuntu 22.04
* ROS2: Humble
* Carla Simulator: 0.9.15

Install client library

* download python package of client library for ubuntu22.04 in [release(carla-0.9.15-ubuntu-22.04)](https://github.com/gezp/carla_ros/releases/)

```
pip3 install carla-0.9.15-cp310-cp310-linux_x86_64.whl
```

Build `carla-simulator/ros-bridge`
```bash
# cd workspace/src
git clone --recurse-submodules https://github.com/peeradonmoke2002/carla_ros.git
# install dependencies
cd ..
rosdep install --from-paths src --ignore-src -r
pip3 install pygame
# compile
colcon build --symlink-install
```

### Reference
* [carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)
* [gezp/carla_ros](https://github.com/gezp/carla_ros)