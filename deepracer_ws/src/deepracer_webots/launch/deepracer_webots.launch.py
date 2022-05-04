import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
# from webots_ros2_utils.urdf_spawner import URDFSpawner, get_webots_driver_node
# from webots_ros2_utils.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
import xacro
import time

def generate_launch_description():

    package_dir = get_package_share_directory('deepracer_webots')
    description_dir = get_package_share_directory('deepracer_description')

    default_rviz_config_path = os.path.join(description_dir, 'rviz', 'urdf_config.rviz')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'robot.urdf')).read_text()

    xacro_path = os.path.join(description_dir, 'models', 'xacro', 'deepracer.xacro')
    deepracer_description: str = xacro.process_file(xacro_path).toxml()
    deepracer_description = deepracer_description.replace("package://meshes", "package://deepracer_description/meshes")

    with open(f"/mnt/d/DeepRacer/CAIRORacer/deepracer-{time.time():0.0f}.urdf.xml", "w") as urdf_file:
        urdf_file.write(deepracer_description)

    # spawn_deepracer_robot = URDFSpawner(
    #     name='deepracer',
    #     robot_description=deepracer_description,
    #     # shell=True
    # )

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'tutorial_world.wbt')
    )

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            { 'robot_description': robot_description }
        ]
    )

    camera_shim = Node(
        package='deepracer_webots',
        executable='camera_shim',
        output='screen',
        parameters=[
            { 'camera_topics': ['/Agent/zed_camera_right_sensor', '/Agent/zed_camera_left_sensor'] }
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{ 'robot_description': deepracer_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_frame_publisher',
        output='log',
        arguments=['0', '0', '0.023249', '0', '0', '0', 'odom', 'map']
    )

    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     output='screen',
    #     parameters=[os.path.join(package_dir, 'config/ekf.yaml'), { 'use_sim_time': False }]
    # )

    robot_localization_node = Node(
        package='deepracer_webots',
        executable='odom_tf_broadcaster',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    # ros2_supervisor = Ros2SupervisorLauncher(output='log', respawn=False)

    # def test_handler(event, next_action):
    #     print('Test Handler!')
    #     print(event)
    #     return next_action

    # e = launch.events.execution_complete.ExecutionComplete

    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                description='Absolute path to rviz config file'),

        webots,
        # ros2_supervisor,

        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessStart(
        #         target_action=ros2_supervisor,
        #         on_start=lambda event, context: test_handler(event, spawn_deepracer_robot)
        #     )
        # ),

        # spawn_deepracer_robot,
        my_robot_driver,
        camera_shim,
        static_tf,
        joint_state_publisher_node,
        robot_state_publisher_node,
        robot_localization_node,
        rviz_node

        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessIO(
        #     # event_handler=launch.event_handlers.OnExecutionComplete(
        #         target_action=spawn_deepracer_robot,
        #         on_stdout=lambda event: get_webots_driver_node(event, my_robot_driver),
        #         # on_completion=lambda event, context: get_webots_driver_node(event, my_robot_driver),
        #     )
        # ),

        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=webots,
        #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        #     )
        # )
    ])

