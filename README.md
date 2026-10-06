# Welcome to GalaxSea's code base!

This repo is the entire ROS 2 workspace (`galaxsea_ws`). Packages live in `src/`:

| Path | What it is |
| --- | --- |
| `src/galaxsea` | Our boat code (controllers, lidar preprocessing, custom Gazebo worlds/models) |
| `src/ROS-TCP-Endpoint` | [Unity ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) (`main-ros2`), as a git submodule, for talking to the Unity simulator |

# Installation

The `roboboat-docker` development image already ships a clone of this workspace at `/root/galaxsea_ws`. To clone it yourself, include the submodules:
```shell
cd /root
git clone --recurse-submodules https://github.com/RoboSub-UTD/galaxsea_ws
```

If you already cloned without `--recurse-submodules`, fetch the submodules with
```shell
git submodule update --init --recursive
```

# Setup
**important:** run these from the workspace root (`galaxsea_ws`), not from `src`
1. Build the workspace
```shell
colcon build --merge-install
```
2. Source the installation
```shell
. install/setup.bash
```

# Simulator + Controller

1. Launch the gazebo simulation using
```shell
ros2 launch vrx_gz competition.launch.py
```
After it launches, you should see a Gazebo GUI with a boat on the water. If you don't, try running 
```shell
xhost +
```
in a new terminal in your host computer. If it still fails, check your run script for allowing displays.

2. Run the keyboard boat controller using
```shell
ros2 run galaxsea boat_controller
```
Using waxd you can move the boat around the lake

# Unity Simulator (ROS-TCP-Endpoint)

Start the endpoint the Unity simulator connects to:
```shell
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

# Setting up Custom Worlds

1. Add the path to the custom models and worlds to $GZ_SIM_RESOURCE_PATH

```shell
export GZ_SIM_RESOURCE_PATH="/root/galaxsea_ws/src/galaxsea/custom_simulations/worlds:/root/galaxsea_ws/src/galaxsea/custom_simulations/models:$GZ_SIM_RESOURCE_PATH"
```

Make sure to put your path in case it is different

2. Launch the new world using 
```shell
ros2 launch vrx_gz competition.launch.py world:=task_one
```
change task_one to whatever the world is called
