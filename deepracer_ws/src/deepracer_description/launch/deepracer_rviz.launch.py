# Adapted from https://automaticaddison.com/how-to-load-a-urdf-file-into-rviz-ros-2/
# This actually won't work; rviz doesn't like our URDF file.
# It says that there aren't transforms defined between base_link and everything else

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare(package='deepracer_description')

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_basic_settings.rviz')

    default_urdf_model_path = os.path.join(pkg_share, 'models/deepracer.xacro')

    gui = LaunchConfiguration('gui')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot URDF file'
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description="Absolute path to the RVIZ config file"
    )
    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Flab to enable joint_state_publisher_gui'
    )
    declare_use_robot_state_publisher_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Flag to enable the robot state publisher'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation time if true'
    )

    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        
    )