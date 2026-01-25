# Unity_ROS2_Robot_Simulator
English | [日本語](README-ja.md)

A Unity-based robot simulator that integrates with ROS2 (Robot Operating System 2), providing a high-fidelity visual environment for robotics simulation and development.

A sample project for a typical two-wheeled mobile robot can be found [here](https://github.com/REACT-ROBOT/Unity_ROS2_sample).

## Overview
This project enables robot simulation in Unity with ROS2 communication capabilities, allowing robotic algorithms to be tested in realistic virtual environments before deployment on physical hardware.

## Features
- Realistic physics simulation using Unity's physics engine
- ROS2 integration for standard robotics communication
- Support for various robot models and sensor types
- Customizable environments for different testing scenarios

## Requirements
- Unity 6000.0.47f1 LTS or newer
- ROS2 (Humble or newer)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [URDF-Importer](https://github.com/Unity-Technologies/URDF-Importer)
- [UnitySensors](https://github.com/Field-Robotics-Japan/UnitySensors)
- [UnitySensorsROS](https://github.com/Field-Robotics-Japan/UnitySensors)
Note: UnitySensors and UnitySensorsROS are used in modifid version now.

## Installation
1. Clone this repository
2. Open the project in Unity
3. (Optional) Build the application.

## Usage
1. Run the built applicaion or the project from Editor.

2. Run TCP Connector from below command.
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
   ```

2. Set up your robot model and sensors
   ```bash
   ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{ name: '<YOURROBOTNAME>', allow_renaming: false, uri: '/your/urdf/path/robot.urdf', resource_string: '', entity_namespace: '', initial_pose: { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' }, pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }"
   ```

3. Start the simulation
   ```bash
   ros2 service call /set_simulation_state simulation_interfaces/srv/SetSimulationState "{ state: { state: 1 } }"
   ```

Note: The services are based on [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces)

## License
This project is licensed under the Apache 2.0 License - see the LICENSE file for details.

## Acknowledgements

I would like to thank the following open-source projects that helped this repository.

* **[dbrizov/NaughtyWaterBuoyancy](https://github.com/dbrizov/NaughtyWaterBuoyancy)**: used for **buoyancy calculations** in this project.
* **[MARUSimulator/marus-core](https://github.com/MARUSimulator/marus-core)**: referenced for approaches on applying **hydrodynamic parameters beyond buoyancy**.
* **[gasgiant/Aircraft-Physics](https://github.com/gasgiant/Aircraft-Physics)**: referenced for approaches on applying **aerodynamic parameters**.
