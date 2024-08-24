
# ROS2 Nav2 Custom Robot Project

This ROS2 project is aimed at implementing Nav2 (Navigation2) on a custom robot platform. Below is a brief overview of the project structure and instructions for launching the simulation, mapping, and navigation components.

## Project Structure

### 1. Description Folder

- **URDF Models**: This folder contains the URDF (Unified Robot Description Format) models that define the robot's physical and visual properties.
    - **robot_core.xacro**: Contains the main robot description, including all visual and collision tags, sensors, and actuators for the robot.
    - **inertial_macros.xacro**: Provides details on the inertial properties of the different robot components. Proper inertial configuration helps in realistic simulation and dynamics calculation.

### 2. Config Folder

- **my_controllers.yaml**: Configuration file for `diff_drive_controller` and `joint_state_publisher`. This file is crucial for defining how the robot's wheels and joints are controlled.
  
- **mapper_params_online_async.yaml**: Configuration file for the SLAM Toolbox. It contains parameters for asynchronous mapping using SLAM, ensuring the robot can navigate and map environments simultaneously.

### 3. Launch Folder

- **launch_sim.launch.py**: This launch file initializes the Gazebo simulation environment with the robot model loaded.
  
- **online_async_launch.py**: Launches the SLAM Toolbox for mapping and localization using the defined parameters.

## URDF Design

The URDF files define the robot's physical structure, sensors, and actuators. Here’s a breakdown:

1. **robot_core.xacro**:
   - Defines the robot's base link, wheel configuration, and sensor mounts.
   - Visual tags include color, texture, and shape definitions for rendering in simulation.
   - Ensures compatibility with Gazebo and RViz for visualization.

2. **inertial_macros.xacro**:
   - Specifies inertial properties like mass, inertia tensor, and center of mass.
   - These properties are crucial for simulating realistic physics and dynamics in Gazebo.

## Mapping with SLAM

For mapping, this project utilizes the SLAM Toolbox. SLAM (Simultaneous Localization and Mapping) is essential for creating a map of the environment while tracking the robot's location in that map.

### Launching Mapping

To launch the mapping component using the SLAM Toolbox, run the following command:

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
```

After launching the mapping component using the SLAM Toolbox, your robot will begin building a map of the environment while localizing itself within that map. Below, we outline additional steps and commands to manage the mapping process effectively and provide insights into the navigation setup using Nav2.

### Running and Managing Mapping

1. **Visualize the Mapping Process**:
   - To observe the mapping in real-time, you can use RViz, a 3D visualization tool for ROS. Open a new terminal and launch RViz with the appropriate configuration file:

     ```bash
     ros2 launch my_bot rviz_launch.py
     ```

   - This command will start RViz with the configuration set up for visualizing the SLAM process. You'll see the robot's live position and the map being created as the robot navigates through the environment.

2. **Save the Map**:
   - Once the mapping process is complete, or you want to save the current state of the map, use the following command to save the map to a file:

     ```bash
     ros2 run nav2_map_server map_saver_cli -f <map_name>
     ```

   - Replace `<map_name>` with the desired name for your map file. This will generate a `.pgm` image file of the map and a `.yaml` file that contains metadata.

## Navigation with Nav2

Nav2 (Navigation2) is used to plan paths and navigate autonomously. Once you have a map, you can utilize Nav2 to guide the robot to specific goals within the mapped environment.

### Setting Up Nav2

1. **Nav2 Configuration Files**:
   - Before launching Nav2, ensure you have properly configured the required YAML files for costmaps, planners, and behavior trees. These files are usually placed in the `config` folder. Some key configuration files include:
     - `nav2_params.yaml`: Defines parameters for the global and local costmaps, planners, and recovery behaviors.
     - `local_costmap_params.yaml`: Configuration for the local costmap, which helps the robot avoid obstacles in its immediate vicinity.
     - `global_costmap_params.yaml`: Configuration for the global costmap, used for global path planning.

2. **Launching Nav2**:
   - To start the Nav2 stack and begin navigation, use a launch file specifically designed for navigation. An example command to launch Nav2 is:

     ```bash
     ros2 launch my_bot nav2_launch.py use_sim_time:=true
     ```

   - This command will start all necessary Nav2 nodes, including the `amcl` (Adaptive Monte Carlo Localization) for localization, planners, controllers, and the lifecycle manager.

3. **Sending Navigation Goals**:
   - Once Nav2 is running and the robot is localized, you can send navigation goals using RViz or command line. To send a goal in RViz:
     - Click on the `2D Nav Goal` button in RViz’s toolbar.
     - Click and drag on the map to set the target position and orientation.

   - Alternatively, use the command line to send a goal:

     ```bash
     ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
     ```

   - Adjust the `x`, `y`, and `orientation` values to the desired target location and orientation.

## Monitoring and Debugging

- **Checking Node Status**:
  Use the following command to check the status of all ROS2 nodes, ensuring everything is running as expected:

  ```bash
  ros2 node list
  ```

- **Viewing Logs**:
  To view logs for debugging purposes, use:

  ```bash
  ros2 launch my_bot nav2_launch.py --log-level debug
  ```

- **RQT Tools**:
  Utilize RQT plugins such as `rqt_graph` to visualize node connections and `rqt_console` to view logging output.

## Additional Tips

1. **Tuning Parameters**: Fine-tuning parameters in `nav2_params.yaml` and other configuration files is crucial for optimal performance. Experiment with parameters related to planner settings, costmap sizes, inflation layers, and sensor update rates.

2. **Testing in Simulation**: Before deploying to a physical robot, thoroughly test your configurations in a simulated environment using Gazebo to identify potential issues.

3. **Updating Maps**: If the environment changes, update your maps by relaunching the mapping process and saving a new map. Keep track of different versions of maps for various environments.

## Conclusion

This project provides a solid foundation for implementing autonomous navigation using ROS2 and Nav2 on a custom robot platform. Feel free to explore and modify the project as needed for your specific requirements. Happy coding! 🤖🚀
