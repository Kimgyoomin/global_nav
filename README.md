# Global Navigation Scripts

This repository contains MATLAB scripts for 3D mapping and path planning using a Hybrid A\* algorithm. Robot models are provided through several ROS description packages in the `robots/` directory.

## MATLAB Usage
1. Install MATLAB with the **Robotics System Toolbox**.
2. Start MATLAB and change the working directory to this repository.
3. Run the main script:
   ```matlab
   main
   ```
   The script creates a 3D occupancy map, loads a Unitree robot model, and plans a path using Hybrid A\*.

## Robot Description Packages
Each subfolder under `robots/` is a standard ROS package containing URDF files, meshes, and launch scripts:

- `a1_description` – Unitree A1 quadruped model.
- `aliengo_description` – Unitree Aliengo quadruped model.
- `aliengoZ1_description` – Unitree Aliengo variant Z1 model.
- `b1_description` – Unitree B1 quadruped model.
- `b2_description` – Unitree B2 quadruped model.
- `b2_description_mujoco` – MuJoCo configuration for B2.
- `b2w_description` – Wheeled variant of the B2 robot.
- `dexterous_hand_description` – Unitree dexterous hand model.
- `g1_description` – Unitree G1 humanoid model.
- `go1_description` – Unitree Go1 quadruped model.
- `go2_description` – Unitree Go2 quadruped model.
- `go2w_description` – Wheeled variant of the Go2 robot.
- `h1_description` – Unitree H1 humanoid model.
- `h1_2_description` – Unitree H1_2 humanoid model.
- `laikago_description` – Unitree Laikago quadruped model.
- `z1_description` – Unitree Z1 arm model.

These packages can be built in a ROS catkin workspace to visualize the robots in RViz or other simulators.

## Dependencies
- ROS with `catkin` tools.
- ROS packages such as `roslaunch`, `robot_state_publisher`, `rviz`, `joint_state_publisher_gui`, and `gazebo` (see each package.xml for details).
- MATLAB with Robotics System Toolbox for running the planning scripts.

## Building the Catkin Workspace
1. Create a new workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone <this repository or individual robot packages>
   ```
2. Build and source the workspace:
   ```bash
   cd ~/catkin_ws
   catkin build
   source devel/setup.bash
   ```
3. Launch a robot description to verify:
   ```bash
   roslaunch go2_description go2_rviz.launch
   ```

