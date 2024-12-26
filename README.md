# ROS2 Nav2 Custom Robot Project 🚀🤖

This ROS2 project is aimed at implementing Nav2 (Navigation2) on a custom robot platform. Below is a brief overview of the project structure and instructions for launching the simulation, mapping, and navigation components. 🎯📡
![Gazebo Environment with bot loaded in it.](https://github.com/user-attachments/assets/950eec61-5247-4c34-b5e3-df00263b550d)

## Project Structure 📂

### 1. Description Folder 📄

- **URDF Models**: This folder contains the URDF (Unified Robot Description Format) models that define the robot's physical and visual properties. 🛠️
    - **robot_core.xacro**: Contains the main robot description, including all visual and collision tags, sensors, and actuators for the robot. 🎨⚙️
    - **inertial_macros.xacro**: Provides details on the inertial properties of the different robot components. Proper inertial configuration helps in realistic simulation and dynamics calculation. 🧮🔧

### 2. Config Folder ⚙️

- **my_controllers.yaml**: Configuration file for `diff_drive_controller` and `joint_state_publisher`. This file is crucial for defining how the robot's wheels and joints are controlled. 🛞📏

- **mapper_params_online_async.yaml**: Configuration file for the SLAM Toolbox. It contains parameters for asynchronous mapping using SLAM, ensuring the robot can navigate and map environments simultaneously. 🗺️🤔

### 3. Launch Folder 🚀

- **launch_sim.launch.py**: This launch file initializes the Gazebo simulation environment with the robot model loaded. 🖥️🏞️

## URDF Design 🛠️

The URDF files define the robot's physical structure, sensors, and actuators. Here’s a breakdown:  

1. **robot_core.xacro**:
   - Defines the robot's base link, wheel configuration, and sensor mounts. 🛠️  
   - Visual tags include color, texture, and shape definitions for rendering in simulation. 🌈  
   - Ensures compatibility with Gazebo and RViz for visualization. 🖼️  

2. **inertial_macros.xacro**:
   - Specifies inertial properties like mass, inertia tensor, and center of mass. 🧘‍♂️  
   - These properties are crucial for simulating realistic physics and dynamics in Gazebo. 🛞🧮  

