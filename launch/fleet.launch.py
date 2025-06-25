import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction, LogInfo, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import ExecuteProcess

import yaml
import tempfile




# This function generates a temporary SDF file for the robot model.
# It reads a template SDF file, replaces the placeholder with the robot name,
# and writes the filled content to a temporary file. It allow the use of odometric plugin with good frames
def generate_model_file(robot_name, model_template_path):
    with open(model_template_path, "r") as f:
        sdf_template = f.read()
    sdf_filled = sdf_template.replace("{{ name }}", robot_name)
    tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".sdf", mode='w')
    tmp_file.write(sdf_filled)
    tmp_file.close()    
    return tmp_file.name





# This function loads the fleet configuration from a YAML file.
def load_config():
    package_share_directory = get_package_share_directory('multirobot_gz_simulator')
    config_file = os.path.join(package_share_directory, 'config', 'fleet_config.yaml')
    with open(config_file, 'r') as f:
        return yaml.safe_load(f)
    





#  This function spawns a robot in the Gazebo world and sets up the ROS-Gazebo bridge for the robot's topics.
#  It reads the robot's configuration from a YAML file and replaces placeholders in the topic names
#  with the robot's namespace and the world name.
def spawn_robot_generic(context, namespace, world, config_path):
    robot_ns = context.perform_substitution(namespace)
    world_name = context.perform_substitution(world)
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    bridge_args = []
    remappings = []
    for topic in config['topics']:
        raw_topic = topic['topic']
        ros_type = topic['type']
        gz_type = topic['gz_type']
        direction = topic.get('direction', 'both')
        remap = topic.get('remap', raw_topic)
        # Replace placeholders
        raw_topic = raw_topic.replace("{{ns}}", robot_ns).replace("{{world}}", world_name)
        remap = remap.replace("{{ns}}", robot_ns).replace("{{world}}", world_name)
        if direction == "ros_to_gz":
            bridge_args.append(f"{raw_topic}@{ros_type}]{gz_type}")
        elif direction == "gz_to_ros":
            bridge_args.append(f"{raw_topic}@{ros_type}[{gz_type}")
        else:
            bridge_args.append(f"{raw_topic}@{ros_type}]{gz_type}")
            bridge_args.append(f"{raw_topic}@{ros_type}[{gz_type}")
        remappings.append((raw_topic, remap))

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{robot_ns}_bridge",
        arguments=bridge_args + ["--ros-args", "--log-level", "warn"],
        parameters=[
            {
                f"qos_overrides./{robot_ns}/tf_static.publisher.durability": "transient_local"
            }
        ],
        remappings=remappings,
        output="log"
    )

    return [LogInfo(msg=[f"Spawning bridge for robot: {robot_ns} in world: {world_name}"]), bridge_node]





def generate_launch_description():
    
    ##########################################################################################
    ## Read the configuration file
    ##########################################################################################
    fleet_config = load_config()
    world_name = fleet_config['world_name']
    world_file = fleet_config['world_file']
    robot_list = fleet_config['robots']
    robot_namespace = fleet_config['robot_ns']  
    


    ##########################################################################################
    ## Launch the world in Gazebo
    ##########################################################################################
    # get package with all ressources
    package_share_directory = get_package_share_directory('multirobot_gz_simulator')

    # get the world launch file argument
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(package_share_directory, 'worlds', world_file),
        description='Path to the world file to load into Gazebo'
    )

    # indicate what is "model://"" path to Gazebo
    os.environ["GAZEBO_MODEL_PATH"] = (os.environ.get("GAZEBO_MODEL_PATH", "") + ":" + os.path.join(package_share_directory, "models"))

    # get the launch command with the selected world file
    world = LaunchConfiguration('world', default=world_name)
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share_directory, 'launch', 'load_world.launch.py')),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
        }.items(),        
    )



    ##########################################################################################
    ## Prepare the robot fleet to be spawned
    ##########################################################################################
  
    # prepare the automatic remapping for each robot provided in the list    
    declare_args = []
    opaque_functions = []
    launch_nodes_robots = []

    for i, robot in enumerate(robot_list, start=1): #i, (name, r_model, r_pose) in enumerate(zip(robot_names, robot_models, robot_poses), start=1):
        name = robot['name']
        type = robot['type']
        pose = robot['pose']
        
        arg_name = f"robot_ns_{i}"
        declare_args.append(
            DeclareLaunchArgument(
                arg_name,
                default_value=name,
                description=f"Namespace du robot {i}"
            )
        )
        robot_ns = LaunchConfiguration(arg_name)

        package_share_directory = get_package_share_directory('multirobot_gz_simulator')
        robot_config_dir = os.path.join(package_share_directory, 'config')
        config_path = os.path.join(robot_config_dir, f"{type}.yaml")

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Le fichier YAML pour le type de robot '{type}' est introuvable : {config_path}")
                
        opaque_functions.append(
            OpaqueFunction(
                function=spawn_robot_generic,
                args=[robot_ns, world, config_path]
            )
        )
        model_template_path = os.path.join(package_share_directory, "models", type, "model.sdf")
        model_sdf_path = generate_model_file(name, model_template_path)

        spawn_node = Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{name}",
            arguments=[
                "-name", name,
                "-x", str(pose[0]),
                "-y", str(pose[1]),
                "-z", str(pose[2]),
                "-R", str(pose[3]),  # Roll
                "-P", str(pose[4]),  # Pitch
                "-Y", str(pose[5]),  # Yaw
                "-file", model_sdf_path,                
            ],
            output="log"
        )
        launch_nodes_robots.append(spawn_node)


    ##########################################################################################
    ## TF relay : merge all the namespaced TFs into a single TF tree
    ##########################################################################################
    tf_relay = Node(        
        package="tf_relay",
        executable="relay",        
        output="log",
        on_exit=Shutdown(),
        arguments=[robot_namespace, str(len(robot_list))],  # namespace and number of agents
    )

    
    ##########################################################################################
    ## Launch description
    ##########################################################################################
    return LaunchDescription([
        # launch the world in Gazebo
        world_file_arg, gz_sim_launch,
        
        # launch the robots fleet in Gazebo     
        *declare_args,        
        *opaque_functions,     
        *launch_nodes_robots,
    
        # launch the TF relay to merge all the TFs
        tf_relay,
    ])