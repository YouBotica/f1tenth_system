#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import datetime

def generate_launch_description():
    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    use_rallycar_hardware = LaunchConfiguration('use_rallycar_hardware')
    record_data = LaunchConfiguration('record_data')
    
    # Generate timestamp for rosbag filename
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")
    
    # Declare launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz')
    
    declare_use_rallycar_hardware_cmd = DeclareLaunchArgument(
        'use_rallycar_hardware',
        default_value='true',
        description='Whether to start rallycar hardware')
        
    declare_record_data = DeclareLaunchArgument(
        'record_data',
        default_value='true',
        description='Record comprehensive data for analysis'
    )
    
    # Include rallycar hardware launch file
    rallycar_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rallycar'),
                'launch',
                'rallycar_hardware.launch.py'
            ])
        ]),
        condition=IfCondition(use_rallycar_hardware)
    )
    
    # Scan matcher node
    scan_matcher_node = Node(
        package='yuwei_scan_matching',
        executable='scan_matcher',
        name='scan_matcher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        remappings=[
            ('/scan', '/scan'),
            ('/scan_match_location', '/scan_match_pose')
        ]
    )
    
    # RViz node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # ROSbag recording
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', f"/home/nvidia-car9/rallycar_rosbag/rallycar_data_{timestamp}", '-a'
        ],
        condition=IfCondition(record_data),
        output='screen'
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_rallycar_hardware_cmd)
    ld.add_action(declare_record_data)  # This was missing
    
    # Add nodes and processes
    ld.add_action(rallycar_hardware_launch)
    ld.add_action(scan_matcher_node)
    ld.add_action(rviz_node)
    ld.add_action(rosbag_record)  # This was missing
    
    return ld
