# Multimodal Interaction and Following for Human-Robot Companionship with Large Language Models

These packages were developed by [Thanh](https://sites.google.com/view/vuthanhcdt/home) from the [Networked Robotic Systems Laboratory](https://sites.google.com/site/yenchenliuncku). If you use any packages from this repository, please cite this repository and our team.

---

## Overview

This project is built on the Agilex Scout Mini Omni platform. The directory structure is as follows:
```
exhibition/
├── genbot                          // Original packages
│   ├──  scout_ros2                 // Genbot-specific ROS2 packages
|   |   ├── scout_base              // Core functions for Genbot
|   |   ├── scout_msgs              // Message definitions for Genbot
|   |   ├── scout_simulation        // Simulation environment for Genbot
|   |   ├── ugv_sdk                 // Data transmission protocol for Genbot
|   |   ├── actor_control           // Control actor on Gazebo
|   |	├── velodyne                // Velodyne-related packages
|   ├── amfitrack                   // Amfitrack packages
├── mppi_controller                 // Human-companion controller
├── README.md

```

## Install Dependent ROS Packages

Before proceeding, ensure you have the following installed and configured:  
- **Jetson 6** with a compatible **JetPack** and **CUDA** version (required for Jetson Orin)  
- **[ZED X Drivers](https://www.stereolabs.com/en-tw/developers/drivers)** and **[ZED X SDK](https://www.stereolabs.com/en-tw/developers/release)**  
- **[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)**  

This project has been thoroughly tested on:  
- **Ubuntu 24.04** with **ROS2 Jazzy** (on the host computer)  
- **JetPack 6.2** (on the robot)  

For optimal compatibility, it is highly recommended to use this setup. To install the required ROS packages, run the following command:  


```bash
sudo apt-get install ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-teleop-twist-joy \
  ros-$ROS_DISTRO-teleop-twist-keyboard ros-$ROS_DISTRO-laser-proc \
  ros-$ROS_DISTRO-urdf ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-compressed-image-transport ros-$ROS_DISTRO-rqt\
  ros-$ROS_DISTRO-interactive-markers \
  ros-$ROS_DISTRO-slam-toolbox\
  ros-$ROS_DISTRO-rqt ros-$ROS_DISTRO-rqt-common-plugins\
  ros-$ROS_DISTRO-sophus\
  ros-$ROS_DISTRO-robot-localization\
  ros-$ROS_DISTRO-realsense2-camera\
  ros-$ROS_DISTRO-realsense2-description\
  build-essential git cmake libasio-dev\
  ros-$ROS_DISTRO-tf2-geometry-msgs\
  ros-$ROS_DISTRO-eigen-stl-containers\
  ros-$ROS_DISTRO-ament-cmake-clang-format\
  ros-$ROS_DISTRO-nmea-msgs\
  ros-$ROS_DISTRO-mavros\
  ros-$ROS_DISTRO-navigation2\
  ros-$ROS_DISTRO-nav2-bringup\
  ros-$ROS_DISTRO-bondcpp\
  ros-$ROS_DISTRO-ompl\
  ros-$ROS_DISTRO-pcl-ros\
  ros-$ROS_DISTRO-sensor-msgs-py\
  ros-$ROS_DISTRO-tf2-tools\
  ros-$ROS_DISTRO-robot-state-publisher\
  ros-$ROS_DISTRO-ros-core\
  ros-$ROS_DISTRO-geometry2\
  ros-$ROS_DISTRO-tf2-sensor-msgs\
  ros-$ROS_DISTRO-spatio-temporal-voxel-layer\
  libompl-dev\
  xterm\
  ros-$ROS_DISTRO-zed-msgs\
  libpcap-dev\
  ros-$ROS_DISTRO-plotjuggler-ros\
  ros-$ROS_DISTRO-mola \
  ros-$ROS_DISTRO-mola-state-estimation \
  ros-$ROS_DISTRO-mola-lidar-odometry\
  ros-${ROS_DISTRO}-tf2-geometry-msgs
```

## Install Genbot Packages

Run the following commands to set up the workspace and install the required packages:
```bash
mkdir -p ~/exhibition_ws/src
cd ~/exhibition_ws/src/
git clone git@github.com:vuthanhcdt/exhibition.git
cd ~/exhibition_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo "source ~/exhibition_ws/install/setup.bash" >> ~/.bashrc
cd ~/exhibition_ws/src/exhibition
pip3 install python-can
uv init ai_agent --bare
lk app env -w
```

## Simulation 
```bash
ros2 launch scout_simulation exhibition.launch.py 
ros2 launch actor_control actor_control.launch.py config:=s1_params.yaml
ros2 launch controller controller_following.launch.py 
```


#### Gazebo GUI Tip
```bash
# Follow a model in the GUI (e.g., a model named "human")
gz service -s /gui/follow --reqtype gz.msgs.StringMsg  --reptype gz.msgs.Boolean  --timeout 2000   --req 'data: "human"'
# Set camera offset relative to the followed model
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d  --reptype gz.msgs.Boolean --timeout 2000  --req 'x: -3  y: 0  z: 2'
```

## Livekit

Install Livekit packages:

```bash
uv add \
  "livekit-agents[openai]~=1.2" \
  "livekit-agents[google]~=1.2" \
  "livekit-plugins-noise-cancellation~=0.2" \
  "python-dotenv" \
  "lark-parser" \
  "empy" \
  "livekit-agents[anam]~=1.2" \
  "livekit-agents[hedra]~=1.2" \
  "pyyaml"
```

## Experiment ROS2 Nodes

Launch experiment nodes:

```bash
# Experiment environment(MID360,scout_base,optimal_point,cost_map)
ros2 launch scout_simulation robot_experiment.launch.py

# Lidar
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# ZED camera (USB-CAN)
ros2 launch zed_wrapper zed_camera.launch.py camera_name:=zed_gimbal camera_model:=zedx serial_number:=49490823 publish_map_tf:=false publish_tf:=false body_tracking:=false object_detection:=true

# Localization
ros2 launch fast_lio_localization localization.launch.py config_file:=mid360.yaml pcd_map_topic:=cloud_pcd map:=/home/genbot1/fastlio_localization_ws/src/FAST_LIO_LOCALIZATION2/PCD/7f.pcd rviz:=False

# AMFITrack
ros2 launch amfitrack amfitrack.launch.py

# reID
ros2 launch reId reid_zed.launch.py

# Trajectory Prediction
python3 tra_pre.py --modality traj #[traj, traj+2dbox,  traj+2dpose,  traj+3dbox, traj+3dpose, traj+all]
python3 tra_pre_amfi.py --modality traj 
ros2 launch tra_pre tra_pre.launch.py --modality traj

#MPPI Controller
ros2 launch control_robot control_robot.launch.py
ros2 launch cost_map cost_map.launch.py
ros2 launch optimal_point optimal_point.launch.py
ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py

#PID Controller
ros2 launch controller_ex controller_Jay.launch.py
ros2 launch controller_ex controller.launch.py
ros2 launch controller_ex controller_no_llm.launch.py
ros2 launch controller_ex controller_following.launch.py
ros2 launch controller_ex collision_detection.launch.py 

# AI Agent
uv run agent.py console

#re-id VLM
ros2 launch people_id people_id.launch.py
```

## AI Agent Nodes

To run the AI agents, ensure you are in the Livekit backend and frontend directories, then execute:

```bash

uv run agent_gemini.py dev
uv run agent_Jay.py dev
uv run agent.py console
uv run agent.py dev
uv run agent_avatar.py dev
npm run dev

python3 agent_reId.py
python3 agent_reId_gemini3.py
python3 agent_reId_gpt5.1.py

ros2 run rqt_image_view rqt_image_view
python3 reid_data_logger.py
python3 agent_controller.py
python3 agent_fall_detected.py

```

## Mapping with Fast LIO

```bash
ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_vlm, zed_gimbal]' cam_models:='[zedx,zedx]' cam_serials:='[43870948,49490823]' publish_map_tf:=False disable_tf:=False body_tracking:='[false,true]' object_detection:='[false,false]' 

ros2 launch gimbal_bringup gimbal_bringup.launch.py

ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py 
ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml
rqt
ros2 launch pcd2pgm pcd2pgm_launch.py 
ros2 run nav2_map_server map_saver_cli -f 7f
```

## Navigation and Following with 2D Mapping
```bash
# Experiment environment
ros2 launch scout_simulation robot_experiment.launch.py 
ros2 launch scout_base scout_mini_omni_base.launch.py publish_tf:=false

# Gimbal bringup and ZED multi-camera (Arduino)
ros2 launch gimbal_bringup_arduino gimbal_bringup.launch.py 
ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_vlm, zed_gimbal]' cam_models:='[zedx,zedx]' cam_serials:='[43870948,44820006]' publish_map_tf:=False disable_tf:=False body_tracking:='[false,true]' object_detection:='[false,false]' 

# Gimbal bringup and ZED multi-camera (USB-CAN)
ros2 launch gimbal_bringup gimbal_bringup.launch.py
ros2 launch zed_multi_camera zed_multi_camera.launch.py cam_names:='[zed_vlm, zed_gimbal]' cam_models:='[zedx,zedx]' cam_serials:='[43870948,49490823]' publish_map_tf:=False disable_tf:=False body_tracking:='[false,true]' object_detection:='[false,false]' 

# Velodyne LiDAR (VLP-16)
ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py 

# Localization
ros2 launch fast_lio_localization localization.launch.py config_file:=velodyne.yaml pcd_map_topic:=cloud_pcd map:=/home/genbot1/fastlio_localization_ws/src/FAST_LIO_LOCALIZATION2/PCD/7f.pcd rviz:=False

# Navigation & Control
ros2 launch navigation navigation.launch.py

#Gimbal Tracking (Arduino)
ros2 launch gimbal_bringup_arduino gimbal_tracking.launch.py 
ros2 launch gimbal_bringup_arduino gimbal_tracking_Jay.launch.py 

#Gimbal Tracking (USB-CAN)
ros2 launch gimbal_bringup gimbal_tracking.launch.py 
ros2 launch gimbal_bringup gimbal_tracking_jay.launch.py 

# Controller
ros2 launch controller_ex controller_Jay.launch.py

# Amfitrack
ros2 launch amfitrack amfitrack.launch.py 

# AI Agent
uv run agent.py console
```



## TODO
- [ ] ...
- [ ] ...
- [ ] ...
ros2 bag record -o exp_follow_01 \
    --include-hidden-topics \
    /tf \
    /tf_static \
    /Odometry \
    /cmd_vel \
    /genbot \
    /robot_status \
    /navigate_to_pose/_action/status \
    /plan


ros2 run tf2_tools view_frames
