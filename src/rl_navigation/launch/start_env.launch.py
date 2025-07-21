# #!/usr/bin/env python3

# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def generate_launch_description():
#     # 获取包路径
#     pkg_rl_navigation = get_package_share_directory('rl_navigation')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
#     pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')

#     # 指定 world 文件路径
#     world_file = os.path.join(pkg_rl_navigation, 'worlds', '2_env.world')

#     # 加载 Gazebo 并指定 world
#     gazebo_launch = IncludeLaunchDescription(
#     PythonLaunchDescriptionSource(
#         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')  
#     ),
#     launch_arguments={
#         'world': world_file
#     }.items()
#     )

#     robot_state_publisher_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_turtlebot3_description, 'launch', 'robot_state_publisher.launch.py')
#         )
#     )



#     # 创建 LaunchDescription
#     ld = LaunchDescription()
#     ld.add_action(gazebo_launch)
#     ld.add_action(robot_state_publisher_launch)
#     # ld.add_action(spawn_robot_node)

#     return ld


#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_rl_navigation = get_package_share_directory('rl_navigation')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # 指定 world 文件路径
    world_file = os.path.join(pkg_rl_navigation, 'worlds', '2_env.world')

    # 设置 TurtleBot3 模型（必须）
    set_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    # 启动 Gazebo，加载 world 和小车
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    return LaunchDescription([
        set_model,
        gazebo_launch
    ])
