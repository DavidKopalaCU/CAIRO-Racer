# Stolen from https://navigation.ros.org/setup_guides/urdf/setup_urdf.html

import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    description_dir = get_package_share_directory('deepracer_description')

    default_urdf_model_path = os.path.join(description_dir, 'models', 'xacro', 'deepracer.xacro')
    default_rviz_config_path = os.path.join(description_dir, 'rviz', 'urdf_config.rviz')

    deepracer_description: str = xacro.process_file(default_urdf_model_path).toxml()
    deepracer_description = deepracer_description.replace("package://meshes", "package://deepracer_description/meshes")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{ 'robot_description': deepracer_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='True',
                                description='Flag to enable the joint_state_publisher gui'),
        DeclareLaunchArgument(name='model', default_value=default_urdf_model_path,
                                description='Absolute path to robot URDF file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                description='Absolute path to rviz config file'),

        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
