
# Multirobot GZ Simulator

A ROS 2 Gazebo-based simulation environment for managing and spawning multiple robots with dynamic bridges and configurations.

## Features

- Generic launch system for spawning multiple robot types
- Dynamic ROS-Gazebo bridge creation based on YAML configuration files
- Supports various robot models like `scoutv2`, `nasa_ingenuity`
- Template-based model generation
- Namespace-based remapping for scalable multi-robot simulations

## Folder Structure

```
multirobot_gz_simulator/
├── config/
│   ├── scoutv2.yaml
│   ├── nasa_ingenuity.yaml
│   ├── fleet_config.yaml
├── launch/
│   ├── fleet.launch.py
│   ├── load_world.launch.py
├── models/
│   ├── scoutv2/
│   ├── nasa_ingenuity/
│   ├── [all other models]
├── worlds/
│   ├── moon.sdf
│   ├── marsyard2022.sdf
│   ├── [all other worlds]
```

## YAML Configuration

Each robot type has a corresponding YAML file in `config/` defining its topic bridges:

Example (`scoutv2.yaml`):

```yaml
topics:
  - topic: "/model/{{ns}}/cmd_vel"
    type: "geometry_msgs/msg/Twist"
    gz_type: "gz.msgs.Twist"
    direction: "ros_to_gz"
    remap: "/{{ns}}/cmd_vel"
  # ...
```

The fleet is described by the yaml:

Example `fleet_config.yaml`:

```yaml
world_file: inspection.sdf
world_name: default   		# the one in the world sdf
robot_ns: fleet			# for tf namespace (could be cleaner...)
robots:
  - name: fleet_0
    type: scoutv2
    pose: [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
  - name: fleet_1
    type: nasa_ingenuity
    pose: [1.0, 2.0, 0.5, 0.0, 0.0, 1.57]
```

## Usage

```bash
ros2 launch multirobot_gz_simulator fleet.launch.py
```


## Dependencies

- ROS 2 Humble or newer
- `ros_gz_sim`, `ros_gz_bridge`
- Gazebo Fortress or Garden
- 'tf_relay' from https://github.com/swarmBots-ipa/tf_relay

Make use of models from spaceros_gz_demos

## License

BSD 3-Clause License

Copyright (c) 2025, Damien Vivet, ISAE-SUPAERO
All rights reserved.


## Contact

For questions, feedback, or contributions, feel free to contact:

**Your Name**  
damien [dot] vivet [at] isae-supaero [dot] fr
