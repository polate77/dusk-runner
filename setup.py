import os
from glob import glob

from setuptools import setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Autonomous delivery rover in a simulated city (Gazebo Harmonic)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = my_robot_controller.teleop:main',
            'autonomous_driver = my_robot_controller.autonomous_driver:main',
            'waypoint_navigator = my_robot_controller.waypoint_navigator:main',
            'chase_camera = my_robot_controller.chase_camera:main',
            'capture_frames = my_robot_controller.capture_frames:main',
            'traffic_manager = my_robot_controller.traffic_manager:main',
            'highres_shot = my_robot_controller.highres_shot:main',
            'photo_rig = my_robot_controller.photo_rig:main',
        ],
    },
)
