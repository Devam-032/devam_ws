# ROS2 Nav2 Custom Robot Project

This ROS2 project is aimed at implementing Nav2 (Navigation2) on a custom robot platform. Below is a brief overview of the project structure and instructions for launching the simulation and mapping components.

## Project Structure

### Description Folder

- **URDF Models**: Contains URDF models of the robot.
- **robot_core.xacro**: Declaration of all visual tags for the robot.
- **inertial_macros.xacro**: Contains information regarding the inertials of different components used in the robot.

### Config Folder

- **my_controllers.yaml**: Configuration file for Diff_drive_controller and Jointstate_publisher.
- **mapper_params_online_async.yaml**: Configuration file for SLAM Toolbox.

## Launching the Simulation

To launch the simulation in Gazebo, execute the following command:

```bash
ros2 launch my_bot launch_sim.launch.py
```

## Launching Mapping

To launch the mapping component using SLAM Toolbox, execute the following command:

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
```

Feel free to explore and modify the project as needed for your custom robot platform. Happy coding! 🤖🚀
