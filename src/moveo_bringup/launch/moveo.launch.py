import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set use_sim_time for all nodes
    use_sim_time = SetParameter(name="use_sim_time", value=True)

    # Launch Gazebo simulation
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("moveo_description"),
            "launch",
            "gazebo.launch.py"
        )
    )

    # Launch MoveIt for motion planning
    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("moveo_moveit_config"),
            "launch",
            "demo.launch.py"
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # Add a delay before launching MoveIt to ensure Gazebo is ready
    moveit_with_delay = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[moveit]
    )

    # Return the launch description with use_sim_time, Gazebo and MoveIt (with delay)
    return LaunchDescription([
        use_sim_time,
        gazebo,
        moveit_with_delay
    ])