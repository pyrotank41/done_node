#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """Launch Gazebo with a drone running PX4 communicating over ROS 2."""
    HOME = os.environ.get('HOME')
    WORK_DIR = HOME + "/my_ws"
    print(f"HOME directory= {HOME}; WORK_DIR = {WORK_DIR}")


    PX4_RUN_DIR = HOME + '/tmp/px4_run_dir'
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

    # package_dir = get_package_share_directory('drone_tracker')
    # px4_sitl_gazebo_dir = HOME + "/my_ws/PX4_Autopilot/Tools/sitl_gazebo" 
  
    #world = os.path.join(px4_sitl_gazebo_dir, 'worlds','typhoon_h480.world')
    world = HOME +'/my_ws/sim_ws/worlds/typhoon_h480.world'
    # model = '/home/karan/my_ws/models/typhoon_h480/typhoon_h480.sdf'
    model = HOME +'/my_ws/sim_ws/models/typhoon_h480_custom/typhoon_h480_custom.sdf'
    # my_ws = HOME + "/my_ws/"
    
    # model = os.path.join(my_ws, 'models', 'typhoon_h480', 'typhoon_h480.sdf')
   
    #custom_gazebo_models = os.path.join(blackdrones_description_dir, 'models')
    #px4_init = os.path.join(blackdrones_description_dir, 'PX4-init')

    os.makedirs(PX4_RUN_DIR, exist_ok=True)

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
                               HOME + '/my_ws/PX4-Autopilot/build/px4_sitl_rtps/build_gazebo'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', '$GAZEBO_MODEL_PATH:' \
                                + HOME + '/my_ws/sim_ws/models:' \
                                + HOME + '/my_ws/PX4-Autopilot/Tools/sitl_gazebo/models'),

        SetEnvironmentVariable('PX4_SIM_MODEL', 'typhoon_h480'),

        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('model', default_value=model),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='1'),
        DeclareLaunchArgument('R', default_value='0.0'),
        DeclareLaunchArgument('P', default_value='0.0'),
        DeclareLaunchArgument('Y', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzserver.launch.py']),
            launch_arguments={
                              'world': LaunchConfiguration('world'),
                              'verbose': 'true'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_dir, '/gzclient.launch.py'])
        ),

        ExecuteProcess(
            cmd=[
                'gz', 'model',
                '--spawn-file', 
                LaunchConfiguration('model'),
                '--model-name', 'drone',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y')
            ],
            prefix="bash -c 'sleep 5s; $0 $@'",
            output='screen'),
        
        ExecuteProcess(
            cmd=[
                HOME + '/my_ws/PX4-Autopilot/build/px4_sitl_rtps/bin/px4',
                HOME + '/my_ws/PX4-Autopilot/ROMFS/px4fmu_common/',
                '-s',
                HOME + '/my_ws/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS'
            ],
            cwd=PX4_RUN_DIR,
            output='screen'),
        
        Node(
            package='ros2_aruco',
            namespace='drone',
            executable='aruco_node',
            name='aruco',
        ),


        # ExecuteProcess(
        #     cmd=['micrortps_agent', '-t', 'UDP'],
        #     output='screen'),
        
        # ExecuteProcess(
        #     cmd=[ HOME + '/QGroundControl.AppImage'],
        #     output='screen'),

])