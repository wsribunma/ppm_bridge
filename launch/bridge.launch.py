from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, NotSubstitution, AndSubstitution, OrSubstitution
from launch.actions import DeclareLaunchArgument, Shutdown, LogInfo, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [

    # launch arguments
    DeclareLaunchArgument('sim',
        default_value=['false'],
        description='use with simulation'
    ),

    DeclareLaunchArgument('rviz',
        default_value='false',
        choices=['true', 'false'],
        description='use rviz for gui.'
    ),

    DeclareLaunchArgument('joy',
        default_value='true',
        choices=['true', 'false'],
        description='use joystick'
    ),

    DeclareLaunchArgument('controller',
        default_value='f310',
        choices=['f310', 'ps4', 'taranis'],
        description='which controller you are using'
    ),

    DeclareLaunchArgument('log_level',
        default_value=['warn'],
        description='Logging level'
    ),
]

def generate_launch_description():

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

    joy_throttle = ExecuteProcess(
        cmd=['ros2', 'run', 'topic_tools', 'throttle', 'messages', 'joy', '20'],
        name='joy_throttle',
        output='log',
        on_exit=Shutdown()
    )

    
    bridge = Node(
        package='ppm_bridge',
        output='log',
        executable='node',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('sim')},
            {'coalesce_interval_ms': 50},
            {'autorepeat_rate': 20.0},
            {'deadzone': 0.02},
            {'controller_id' : LaunchConfiguration('controller')}
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

    return LaunchDescription(ARGUMENTS + [
        joy,
        bridge,
        joy_throttle,
        rviz_node,
    ])
