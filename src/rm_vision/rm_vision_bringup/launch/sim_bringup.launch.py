import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    namespace = LaunchConfiguration('namespace', default='blue_standard_robot1')
    use_namespace = LaunchConfiguration('use_namespace', default='true')

    launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

    node_params = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'sim_node_params.yaml')
    
    remappings = [('/tf', '/blue_standard_robot1/tf'),
                  ('/tf_static', '/blue_standard_robot1/tf_static')]
    
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            remappings=remappings,
            namespace=namespace,
            parameters=[{'robot_description': robot_description,
                        'publish_frequency': 1000.0}]
        ),
        ComposableNodeContainer(
            name='camera_detector_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            remappings=remappings,
            composable_node_descriptions=[
                ComposableNode(
                    package='hik_camera',
                    plugin='sim_camera::SimCameraNode',
                    name='sim_camera_node',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        ),
        TimerAction(
            period=1.5,
            actions=[Node(
                    package='rm_serial_driver',
                    executable='sim_serial_driver',
                    name='sim_serial_driver',
                    namespace=namespace,
                    output='both',
                    parameters=[node_params],
                    remappings=remappings,
                    on_exit=Shutdown(),
                    ros_arguments=['--ros-args', '--log-level',
                                'serial_driver:='+launch_params['serial_log_level']],
                    )],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='armor_tracker',
                    executable='armor_tracker_node',
                    name='armor_tracker',
                    namespace=namespace,
                    output='both',
                    emulate_tty=True,
                    parameters=[node_params],
                    remappings=remappings,
                    ros_arguments=['--log-level', 'armor_tracker:='+launch_params['tracker_log_level']],
                    )],
        )
        

    ])

    return LaunchDescription([
        bringup_cmd_group
    ])

