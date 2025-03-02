import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
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
    
    # Add a delay before launching MoveIt
    moveit_with_delay = TimerAction(
        period=4.0,  # Delay in seconds
        actions=[moveit]
    )
    
    # Return the launch description with Gazebo and MoveIt (with delay)
    return LaunchDescription([
        gazebo,
        moveit_with_delay
    ])