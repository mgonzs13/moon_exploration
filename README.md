# moon_exploration

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/ros2_rover
git clone https://github.com/uleroboticsgroup/simple_node.git
git clone https://github.com/mgonzs13/moon_exploration
git clone --recurse-submodules https://github.com/mgonzs13/teb_local_planner.git
cd ~/ros2_ws
rosdep install --from-paths src -r -y
colcon build
```

## Usage

```shell
ros2 launch moon_exploration_bringup moon_exploration.launch.py radius:=10 waypoints_nums:=20 exploration_mode:=spiral nav2_planner:=SmacHybrid nav2_controller:=RPP use_low_moon:=False
```

```shell
ros2 action send_goal /run_moon_exploration moon_exploration_msgs/action/RunMoonExploration {}
```
