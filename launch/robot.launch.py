"""Launch the delivery rover in the procedural city world.

    ros2 launch my_robot_controller robot.launch.py

Useful arguments:
    mode:=nav              waypoint_navigator (default)
    mode:=auto             reactive obstacle-avoidance driver
    mode:=teleop           keyboard teleoperation
    mode:=none             no driver at all
    capture:=false         do not take the three city screenshots
    gz_args:='-r -s ...'   headless server only

The GUI chase camera (chase_camera) always runs: it keeps the Gazebo 3D view
behind the rover so no mouse work is needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'my_robot_controller'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    world_path = os.path.join(share, 'worlds', 'my_city.world')
    urdf_path = os.path.join(share, 'urdf', 'my_robot.urdf')

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # -------------------------------------------------------- arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_args = LaunchConfiguration('gz_args')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the /clock published by Gazebo'),
        DeclareLaunchArgument(
            'gz_args', default_value='-r ' + world_path,
            description='Arguments for gz sim. Use "-r -s <world>" for headless.'),
        DeclareLaunchArgument(
            'mode', default_value='nav',
            description="Driver to run: 'nav', 'auto', 'teleop' or 'none'"),
        DeclareLaunchArgument(
            'capture', default_value='true',
            description='Save three camera stills while driving'),
        DeclareLaunchArgument(
            'traffic', default_value='true',
            description='Spawn moving cars and pedestrians'),
    ]

    # ------------------------------------------------------ Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # The city world takes a moment to load; give the server time to come up.
    spawn_robot = TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description',
                       '-name', 'my_robot',
                       '-x', '0.0', '-y', '-11.0', '-z', '0.03',
                       '-Y', '1.5708'],
            output='screen',
        )],
    )

    # ------------------------------------------- ROS <-> Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Gazebo publishes the clock per world; expose it as ROS /clock.
            '/world/city/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        remappings=[('/world/city/clock', '/clock')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    navigator = Node(
        package=PACKAGE,
        executable='waypoint_navigator',
        name='waypoint_navigator',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'nav'),
    )

    autonomous = Node(
        package=PACKAGE,
        executable='autonomous_driver',
        name='autonomous_driver',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'auto'),
    )

    teleop = Node(
        package=PACKAGE,
        executable='teleop',
        name='teleop',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=LaunchConfigurationEquals('mode', 'teleop'),
    )

    # Keeps the Gazebo 3D view behind the rover (racing-game chase camera).
    chase_camera = Node(
        package=PACKAGE,
        executable='chase_camera',
        name='chase_camera',
        output='screen',
    )

    capture = Node(
        package=PACKAGE,
        executable='capture_frames',
        name='capture_frames',
        parameters=[{
            'output_dir': os.path.join(os.path.expanduser('~'), '.ros', 'captures'),
            'count': 3,
            'spacing': 45.0,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        condition=LaunchConfigurationEquals('capture', 'true'),
    )

    # Moving traffic: static Fuel models teleported along lanes in sim time.
    # It retries until the city's create service is up, so no fixed delay.
    traffic = Node(
        package=PACKAGE,
        executable='traffic_manager',
        name='traffic_manager',
        output='screen',
        condition=LaunchConfigurationEquals('traffic', 'true'),
    )

    return LaunchDescription(declared_arguments + [
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        chase_camera,
        navigator,
        autonomous,
        teleop,
        capture,
        traffic,
    ])
