from setuptools import find_packages, setup
from pathlib import Path  
from glob import glob

package_name = 'rl_navigation'

launch_files = [*Path('launch').glob('*.launch.py'), *Path('launch').glob('*.launch.xml')]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/rl_navigation/launch', ['launch/start_env.launch.py', 'launch/test_env.launch.py',
        #  'launch/bringup_training.launch.py', 'launch/bringup_training.launch.xml', 'launch/bringup_training_instrumented.launch.xml']),
        ('share/' + package_name + '/launch', [str(p) for p in launch_files]),
        ('share/rl_navigation/worlds', ['worlds/2_env.world', 'worlds/4_env.world'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mscrobotics2425laptop14',
    maintainer_email='haoran.yan01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_ddqn = rl_navigation.train_ddqn:train_ddqn', 
            'scan_sanitizer = rl_navigation.scan_sanitizer:main',
            'cmd_guard = rl_navigation.cmd_guard:main',
        ],
    },
)
