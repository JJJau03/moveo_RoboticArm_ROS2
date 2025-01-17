from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    arduinobot_description_dir = get_package_share_directory("moveo_description")
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(arduinobot_description_dir, "urdf", "moveo.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(arduinobot_description_dir).parent.resolve())]
    )

    # Adjust ROS_DISTRO and set physics engine accordingly
    ros_distro = os.getenv("ROS_DISTRO", "jazzy")  # Default to "foxy" if undefined
    is_ignition = "True" if ros_distro == "jazzy" else "False"
    physics_engine = "" if ros_distro == "foxy" else "--physics_engine gz-physics-bullet-featherstone-plugin"

    # Robot description parameter
    # robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]), value_type=str)
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
        arguments=["--ros-args", "--log-level", "debug"]
    )

    # Gazebo launch include
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-v 4 -r empty.sdf {physics_engine}"}.items()
    )

    # Gazebo entity spawner
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "moveo_description"],
        parameters=[{"timeout": 10.0}]  # Adjusting timeout to ensure `robot_description` is available
    )

    # ROS2-Gazebo Bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])
