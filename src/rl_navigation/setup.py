from setuptools import find_packages, setup

package_name = 'rl_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/rl_navigation/launch', ['launch/start_env.launch.py', 'launch/test_env.launch.py']),
        ('share/rl_navigation/worlds', ['worlds/2_env.world'])
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
        ],
    },
)
