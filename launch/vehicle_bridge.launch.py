from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, NotSubstitution, AndSubstitution, OrSubstitution
from launch.actions import DeclareLaunchArgument, Shutdown, LogInfo, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.conditions import LaunchConfigurationEquals, IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer, PushROSNamespace
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
import os

ARGUMENTS = [

    # launch arguments
    DeclareLaunchArgument(
        'sim',
        default_value=['false'],
        description='use with simulation'
    ),

    DeclareLaunchArgument(
        'rviz',
        default_value='false',
        choices=['true', 'false'],
        description='use rviz for gui.'
    ),

    DeclareLaunchArgument(
        'joy',
        default_value='true',
        choices=['true', 'false'],
        description='use joystick'
    ),

    DeclareLaunchArgument(
        'controller',
        default_value='taranis',
        choices=['f310', 'ps4', 'taranis'],
        description='which controller you are using'
    ),

    DeclareLaunchArgument(
        'log_level',
        default_value=['warn'],
        description='Logging level'
    ),

    DeclareLaunchArgument(
        'vehicle',
        default_value= ['nv1'],
        choices=['nv1','nv2','nv3','nv4','nv5'],
        description='vehicle id for namespace use'
    ),
]

def generate_launch_description():
    # ns = LaunchConfiguration('namespace', default = 'nv1')
    
    # ld = LaunchDescription()
    joy = Node(
        package='joy',
        output='log',
        executable='joy_node',
        condition=IfCondition(LaunchConfiguration('joy')),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('sim')},
            {'coalesce_interval_ms': 1},
            {'autorepeat_rate': 20.0},
            {'deadzone': 0.02},
            ],
        on_exit=Shutdown()
    )


    auto_joy_throttle_container = ComposableNodeContainer(
            name='auto_joy_thr',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='topic_tools',
                    plugin='topic_tools::ThrottleNode',
                    name='throttle',
                    parameters=[{
                        'throttle_type':'messages',
                        'input_topic':'auto_joy',
                        'msgs_per_sec': 10.0}
                        ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            output='both',
    )


    joy_throttle_container = ComposableNodeContainer(
            name='joy_thr',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='topic_tools',
                    plugin='topic_tools::ThrottleNode',
                    name='joy_throttle',
                    parameters=[{
                        'throttle_type':'messages',
                        'input_topic':'joy',
                        'msgs_per_sec': 10.0}
                        ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            output='both',
    )

    bridge = Node(
        package='ppm_bridge',
        output='log',
        executable='node',
        name='bridge',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # namespace=LaunchConfiguration('ns_id'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('sim')},
            {'coalesce_interval_ms': 50},
            {'autorepeat_rate': 20.0},
            {'deadzone': 0.02},
            {'controller_id' : LaunchConfiguration('controller')},
            {'vehicle_id': LaunchConfiguration('vehicle')}
            ],
        on_exit=Shutdown()
    )
 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=[
            '-d', [PathJoinSubstitution([FindPackageShare('electrode'), 'config',
            'bridge']), '.rviz'], '--ros-args', '--log-level',
            LaunchConfiguration('log_level')],
        parameters=[{'use_sim_time': LaunchConfiguration('sim')}],
        on_exit=Shutdown(),
    )

    vehicle_description = LaunchDescription(ARGUMENTS+ [
        joy,
        bridge,
        rviz_node,
        joy_throttle_container,
        auto_joy_throttle_container,
    ])

    vehicle_1 = GroupAction(
        actions=[
            PushROSNamespace(LaunchConfiguration('vehicle')),
            vehicle_description
        ]
        )
    
    return LaunchDescription(ARGUMENTS+ [vehicle_1])
