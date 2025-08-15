import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rl_navigation = get_package_share_directory('rl_navigation')
    sim_launch = os.path.join(pkg_rl_navigation, 'launch', 'test_env.launch.py')

    # 训练脚本：以 “python -m rl_navigation.train_ddqn” 方式运行
    start_training = ExecuteProcess(
        cmd=['python3', '-m', 'rl_navigation.train_ddqn'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'}
    )


    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)),
        start_training,
    ])
