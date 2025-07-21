# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     pkg_rl_navigation = get_package_share_directory('rl_navigation')
#     world_file = os.path.join(pkg_rl_navigation, 'worlds', '2_env.world')

#     # 设置 TurtleBot3 模型（必须）
#     set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

#     # 启动 Gazebo
#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
#         ),
#         launch_arguments={
#             'world': world_file,
#             'verbose': 'true'
#         }.items()
#     )

#     return LaunchDescription([
#         set_model,
#         gazebo_launch
#     ])


#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取路径
    pkg_rl_navigation = get_package_share_directory('rl_navigation')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    world_file = os.path.join(pkg_rl_navigation, 'worlds', '2_env.world')
    urdf_file = os.path.join(pkg_turtlebot3_description, 'urdf', 'turtlebot3_burger.urdf')

    # 设置 TurtleBot3 模型（必须）
    set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    # 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 启动 robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # 插入 TurtleBot3
    # spawn_turtlebot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'turtlebot3',
    #         '-topic', 'robot_description',
    #         '-x', '-3', '-y', '0', '-z', '0.01'
    #     ],
    #     output='screen'
    # )

    # spawn_turtlebot = Node(
    # package='gazebo_ros',
    # executable='spawn_entity.py',
    # arguments=[
    #     '-entity', 'turtlebot3',
    #     '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
    #     '-x', '-3', '-y', '0', '-z', '0.01'
    # ],
    # output='screen'
    # )


    return LaunchDescription([
        set_model,
        gazebo,
        robot_state_publisher,
        # spawn_turtlebot
    ])


# # <!-- 内置 TurtleBot3 Burger -->
#     <include>
#       <uri>model://turtlebot3_burger</uri>
#       <pose>-3 0 0.01 0 0 0</pose> <!-- 起点位置 -->
#     </include>
