# **rox**

This package provides provides the bringup, robot description and navigation files for the latest `rox` platform supporting UR10, UR10e, UR5, UR5e and Elite EC66 arms.

## **Launch Command**

To start the simulation:
```sh
ros2 launch rox_bringup bringup_sim_launch.py arm_type:=ur10
```

## **Launch Arguments**

To check available arguments:
```sh
ros2 launch rox_bringup bringup_sim_launch.py --show-args
```

### **Arguments**
```text
'imu_enable':
    Enable IMU - Options: True/False
    (default: 'False')

'd435_enable':
    Enable Realsense - Options: True/False
    (default: 'False')

'arm_type':
    Arm Types:
        Elite Arms: ec66, cs66
        Universal Robotics (UR): ur5, ur10, ur5e, ur10e
    (default: '')

'frame_type':
    Frame type - Options: short/long
    (default: 'short')

'rox_type':
    Robot type - Options: argo/diff/trike/meca
    (default: 'argo')

'use_ur_dc':
    Set this argument to True if you have an UR arm with DC variant
    (default: 'false')

'initial_joint_controller':
    Robot controller to start:
        Elite Arms: arm_controller
        Universal Robotics (UR): joint_trajectory_controller, scaled_joint_trajectory_controller
```

## **Note**
For simulation, set:
```yaml
speed_scaling_interface_name: ""
```
This setting is commented in every UR arm `ur_controllers.yaml` file under `rox_bringup/configs/ur/`.

## **Navigation Commands**

Start navigation:
```sh
ros2 launch rox_navigation navigation.launch.py use_sim_time:=True
```

Start RViz for visualization:
```sh
ros2 launch neo_nav2_bringup rviz_launch.py
```

## **MoveIt2 Simulation**

Start MoveIt2 with Gazebo:
```sh
ros2 launch neo_rox_moveit2 neo_ur_moveit.launch.py arm_type:=ur10 use_sim_time:=True prefix:=ur10 launch_rviz:=True use_gz:=true
```

---

## **Changes in This Update**

### **rox_description**
- Updated `gazebo.xacro` for migration to Modern Gazebo (Ionic).
- Added modular URDF xacros for `ur_arm` and `elite_arm`.
- Refactored `rox.urdf.xacro` to be modular.

### **rox_bringup**
- Added ROS 2 Control and Joint Trajectory Controller configurations for UR and Elite arms.
- Updated `gz_bridge_config.yaml` to support the Modern Gazebo migration and updated /cmd_vel message type.
- Modified `bringup_sim_launch.py` to accommodate the new URDF format and include necessary ROS 2 Control nodes for arms.
- Updated the teleop publish message type for /cmd_vel topic.
- Added model paths to `GZ_SIM_RESOURCE_PATH` env variable in launch file.
- Set use ac/dc parameter in simulation launch file and updated arm position(height) on cabinet for ac variant
- UR arm urdf in `ur_description` package for rolling is different from the iron distro. So this package was  adapted to those changes and appropriate packages were used.

### **General**
- Updated dependencies.
- Packages restucturing.

