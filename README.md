# AKY Model — ROS 2 UGV Task 2

`aky_model` is a complete RViz2-only custom UGV demo for ROS 2 Humble. It contains a four-wheel Xacro robot, all essential links/joints/TF frames, RViz configuration, and a kinematic ROS 2 node driven by keyboard teleoperation.

The project deliberately uses a lightweight visual simulator rather than Gazebo: keyboard commands move the robot in RViz and rotate all four wheels, without requiring physics or hardware.

## Project structure

```text
~/aky_model_ws/
└── src/
    └── aky_model/                         # Git repository and ROS package
        ├── aky_model/ugv_motion.py        # /cmd_vel to odom, TF, joint states
        ├── launch/display.launch.py       # starts all RViz demo nodes
        ├── rviz/aky_model.rviz            # saved RViz view
        ├── urdf/aky_model.urdf.xacro      # custom robot model
        ├── package.xml
        └── setup.py
```

## Architecture

```text
teleop_twist_keyboard ── /cmd_vel ──> aky_motion
                                          ├── /joint_states ──> robot_state_publisher ──> wheel TFs
                                          ├── /odom
                                          └── odom -> base_footprint TF
                                                                              └──> RViz2
```

The expected TF tree is:

```text
odom
└── base_footprint
    └── base_link
        ├── front_left_wheel_link
        ├── rear_left_wheel_link
        ├── front_right_wheel_link
        ├── rear_right_wheel_link
        ├── battery_link
        └── camera_link -> camera_optical_frame
```

## Prerequisites

Use Ubuntu 22.04 with ROS 2 Humble and `zsh`:

```zsh
source /opt/ros/humble/setup.zsh
sudo apt update
sudo apt install -y python3-colcon-common-extensions \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-ros ros-humble-tf2-tools
```

## Build and validate

```zsh
source /opt/ros/humble/setup.zsh
cd ~/aky_model_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select aky_model
source install/setup.zsh
ros2 run xacro xacro src/aky_model/urdf/aky_model.urdf.xacro > /tmp/aky_model.urdf
```

The final command must return without an error before launching RViz.

## Run the rover

Terminal 1 starts the robot, its TF publishers, and RViz:

```zsh
source /opt/ros/humble/setup.zsh
source ~/aky_model_ws/install/setup.zsh
ros2 launch aky_model display.launch.py
```

Terminal 2 controls the rover. Keep this terminal focused while pressing keys:

```zsh
source /opt/ros/humble/setup.zsh
source ~/aky_model_ws/install/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/cmd_vel -p speed:=0.4 -p turn:=1.0 \
  -p repeat_rate:=10.0 -p key_timeout:=0.6
```

| Key | Action |
|---|---|
| `i` | Forward |
| `,` | Reverse |
| `j` / `l` | Turn left / right |
| `u` / `o` | Forward-left / forward-right |
| `k` | Stop |

RViz uses `odom` as its Fixed Frame. The yellow markers added to the wheels make the rotations visible during screenshots and demonstrations.

## Test evidence

In a third sourced terminal, run:

```zsh
ros2 node list
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

To make a short manual test without keyboard input:

```zsh
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Capture evidence of the model in RViz, a turning rover, the teleop terminal, `/joint_states`, and the TF tree.

## GitHub publishing

This repository has meaningful local commits for package setup, the Xacro model, RViz bringup, teleoperation, and documentation. Create an **empty** GitHub repository named `aky_model`, then attach and push it:

```zsh
cd ~/aky_model_ws/src/aky_model
git remote add origin https://github.com/YOUR_GITHUB_USERNAME/aky_model.git
git push -u origin main
git status
git log --oneline -5
```

Do not initialize the GitHub repository with a README, `.gitignore`, or license before its first push. If your instructor supplied a repository URL, use that URL as `origin` instead.

## Notes

- Do not run `joint_state_publisher` alongside this project: `aky_motion` is the sole publisher of `/joint_states`.
- Do not enable `use_sim_time`; there is no `/clock` source in this RViz-only project.
- This package is a virtual kinematic demo, not a driver for physical motors.
