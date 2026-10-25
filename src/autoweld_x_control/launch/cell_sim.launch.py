import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('autoweld_x_control')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg, 'robots', 'cellule.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1. Gazebo avec un monde vide
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items())

    # 2. Publication de la description du robot
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': True}],
               output='screen')

    # 3. Apparition de la cellule dans Gazebo
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'cellule'],
                 output='screen')

    # 4. Pont pour l'horloge de simulation
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                  output='screen')

    # 5. Controleurs, lances l'un apres l'autre
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster',
                          '--controller-manager-timeout', '60'])
    robot_a = Node(package='controller_manager', executable='spawner',
                   arguments=['robot_a_controller',
                              '--controller-manager-timeout', '60'])

    return LaunchDescription([
        gazebo, rsp, bridge, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[robot_a])),
    ])
